#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <motion_capture_tracking_interfaces/msg/named_pose_array.hpp>

// Legacy FastRTPS time sync — not used with uXRCE-DDS (UXRCE_DDS_SYNCT handles this).
// Kept as reference; this block is never compiled unless PX4_ROS_TIMESYNC is defined.
#ifdef PX4_ROS_TIMESYNC
	#include <px4_msgs/msg/timesync.hpp>
#endif

#include <geometry_msgs/msg/pose_stamped.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

// --- Watchdog thresholds ---
// WATCHDOG_TIMEOUT_S: after this long with no Mocap data, stop holding and let EKF2 decide.
static constexpr double WATCHDOG_TIMEOUT_S = 0.2;
// MIN_MOCAP_PERIOD_S: gap larger than this means Mocap has dropped below 30Hz minimum.
// 1/30Hz ≈ 33ms. Only when the gap exceeds this does the timer fill in.
static constexpr double MIN_MOCAP_PERIOD_S = 1.0 / 30.0;

class MocapPX4Bridge : public rclcpp::Node
{
public:
	MocapPX4Bridge() : Node("mocap_px4_bridge"), has_pose_(false) {
		this->declare_parameter("mocap_topic", "/poses");
		this->declare_parameter("px4_topic", "/fmu/in/vehicle_visual_odometry");
		this->declare_parameter("drone_name", "Puck");

		const std::string mocap_topic = this->get_parameter("mocap_topic").as_string();
		const std::string px4_topic = this->get_parameter("px4_topic").as_string();
		drone_name_ = this->get_parameter("drone_name").as_string();

		RCLCPP_INFO(get_logger(), ("mocap_topic: " + mocap_topic).c_str());
		RCLCPP_INFO(get_logger(), ("px4_topic:   " + px4_topic).c_str());

		auto qos = rclcpp::SensorDataQoS();
		poseSub = this->create_subscription<motion_capture_tracking_interfaces::msg::NamedPoseArray>(
			mocap_topic, qos, std::bind(&MocapPX4Bridge::posesCallback, this, _1));
		odomPub = this->create_publisher<px4_msgs::msg::VehicleOdometry>(px4_topic, 10);

		// Watchdog timer fires at 50Hz (every 20ms).
		// It is SILENT when Mocap is healthy — it only publishes when a dropout is detected.
		watchdogTimer_ = this->create_wall_timer(
			20ms, std::bind(&MocapPX4Bridge::watchdogTimerCallback, this));
	}

private:
	// Watchdog: fires every 20ms. Only publishes if Mocap has gone silent
	// (gap > 33ms) AND is still within recovery window (< 500ms).
	void watchdogTimerCallback() {
		if (!has_pose_) {
			return; // Never received a pose — nothing to hold.
		}

		auto now = this->get_clock()->now();
		double age_s = (now - last_received_time_).seconds();

		// Mocap is healthy: a real packet arrived recently. Stay silent.
		if (age_s < MIN_MOCAP_PERIOD_S) {
			return;
		}

		// Dropout has exceeded recovery window. Stop holding — let EKF2 decide.
		if (age_s > WATCHDOG_TIMEOUT_S) {
			RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
				"Mocap dropout: %.2fs since last pose (> %.2fs limit). Stopping hold.",
				age_s, WATCHDOG_TIMEOUT_S);
			return;
		}

		// Dropout detected (33ms < gap < 500ms): republish last known pose.
		// Refresh timestamp to "now" so EKF2 does not reject it as stale.
		RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 200,
			"Mocap dropout detected (%.0fms gap). Holding last pose.", age_s * 1000.0);

		px4_msgs::msg::VehicleOdometry msg = last_odom_msg_;
		uint64_t now_us = uint64_t(now.nanoseconds()) / 1000;
		msg.timestamp = now_us;
		msg.timestamp_sample = now_us;
		odomPub->publish(msg);
	}

	// Pass-through: called on every real Mocap frame. Publishes directly (full Mocap rate)
	// and stores the pose so the watchdog can hold it during a dropout.
	void posesCallback(const motion_capture_tracking_interfaces::msg::NamedPoseArray::SharedPtr msg) {
		for (const auto &named_pose : msg->poses) {
			if (named_pose.name != drone_name_) {
				continue;
			}
			const auto &poseMsg = named_pose.pose;

			RCLCPP_INFO_ONCE(get_logger(), "Received first msg from optitrack.");
			RCLCPP_INFO_ONCE(get_logger(), "P: %f, %f, %f",
				poseMsg.position.x, poseMsg.position.y, poseMsg.position.z);
			RCLCPP_INFO_ONCE(get_logger(), "q: %f, %f, %f, %f",
				poseMsg.orientation.w, poseMsg.orientation.x,
				poseMsg.orientation.y, poseMsg.orientation.z);

			px4_msgs::msg::VehicleOdometry odomMsg;
			odomMsg.pose_frame = odomMsg.POSE_FRAME_FRD;

			// timestamp: wall-clock "now" (what EKF2 uses for fusion timing)
			uint64_t now_us = uint64_t(this->get_clock()->now().nanoseconds()) / 1000;
			odomMsg.timestamp = now_us;
			// timestamp_sample: raw Mocap origin time (preserved for delay analysis)
			odomMsg.timestamp_sample = uint64_t(msg->header.stamp.sec) * 1000000
				+ uint64_t(msg->header.stamp.nanosec) / 1000;

			odomMsg.position[0] = poseMsg.position.x;
			odomMsg.position[1] = -poseMsg.position.y;
			odomMsg.position[2] = -poseMsg.position.z;

			odomMsg.q[0] = poseMsg.orientation.w;
			odomMsg.q[1] = poseMsg.orientation.x;
			odomMsg.q[2] = -poseMsg.orientation.y;
			odomMsg.q[3] = -poseMsg.orientation.z;

			// Publish directly (pass-through at full Mocap rate).
			odomPub->publish(odomMsg);

			// Store for watchdog use during dropout.
			last_odom_msg_ = odomMsg;
			last_received_time_ = this->get_clock()->now();
			has_pose_ = true;
			break;
		}
	}

	rclcpp::Subscription<motion_capture_tracking_interfaces::msg::NamedPoseArray>::SharedPtr poseSub;
	rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr odomPub;
	rclcpp::TimerBase::SharedPtr watchdogTimer_;

	std::string drone_name_;
	px4_msgs::msg::VehicleOdometry last_odom_msg_;
	rclcpp::Time last_received_time_;
	bool has_pose_;
};


int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MocapPX4Bridge>());
	rclcpp::shutdown();
	return 0;
}
