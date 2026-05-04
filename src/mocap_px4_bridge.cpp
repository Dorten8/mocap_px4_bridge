#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <motion_capture_tracking_interfaces/msg/named_pose_array.hpp>

#ifdef PX4_ROS_TIMESYNC
	#include <px4_msgs/msg/timesync.hpp>
#endif

#include <geometry_msgs/msg/pose_stamped.hpp>

using std::placeholders::_1;

using namespace std::chrono_literals;


class MocapPX4Bridge : public rclcpp::Node
{
public:
	MocapPX4Bridge() : Node("mocap_px4_bridge") {
		this->declare_parameter("mocap_topic", "/poses");
		this->declare_parameter("px4_topic", "/fmu/in/vehicle_visual_odometry");
		this->declare_parameter("drone_name", "Puck");

		const std::string mocap_topic = this->get_parameter("mocap_topic").as_string();
		const std::string px4_topic = this->get_parameter("px4_topic").as_string();
		drone_name_ = this->get_parameter("drone_name").as_string();

		RCLCPP_INFO(get_logger(), ("mocap_topic: " + mocap_topic).c_str());
		RCLCPP_INFO(get_logger(), ("px4_topic:   " + px4_topic).c_str());

		poseSub = this->create_subscription<motion_capture_tracking_interfaces::msg::NamedPoseArray>(mocap_topic, 10, std::bind(&MocapPX4Bridge::posesCallback, this, _1));
		odomPub = this->create_publisher<px4_msgs::msg::VehicleOdometry>(px4_topic, 10);
	}

private:
	void posesCallback(const motion_capture_tracking_interfaces::msg::NamedPoseArray::SharedPtr);

	rclcpp::Subscription<motion_capture_tracking_interfaces::msg::NamedPoseArray>::SharedPtr poseSub;
	rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr odomPub;
	std::string drone_name_;
};

void MocapPX4Bridge::posesCallback(const motion_capture_tracking_interfaces::msg::NamedPoseArray::SharedPtr msg){
  for (const auto &named_pose : msg->poses) {
    if (named_pose.name != drone_name_) {
      continue;
    }
    const auto &poseMsg = named_pose.pose;

    RCLCPP_INFO_ONCE(get_logger(), "Recived first msg from optitrack.");
    RCLCPP_INFO_ONCE(get_logger(), "P: %f, %f, %f",
                     poseMsg.position.x, poseMsg.position.y, poseMsg.position.z);
    RCLCPP_INFO_ONCE(get_logger(), "q: %f, %f, %f, %f",
                     poseMsg.orientation.w, poseMsg.orientation.x,
                     poseMsg.orientation.y, poseMsg.orientation.z);

    px4_msgs::msg::VehicleOdometry odomMsg;
    odomMsg.pose_frame = odomMsg.POSE_FRAME_FRD;
    odomMsg.timestamp = uint64_t(msg->header.stamp.sec)*1000000 + uint64_t(msg->header.stamp.nanosec)/1000;
    odomMsg.timestamp_sample = odomMsg.timestamp;

    odomMsg.position[0] = poseMsg.position.x;
    odomMsg.position[1] = -poseMsg.position.y;
    odomMsg.position[2] = -poseMsg.position.z;

    odomMsg.q[0] = poseMsg.orientation.w;
    odomMsg.q[1] = poseMsg.orientation.x;
    odomMsg.q[2] = - poseMsg.orientation.y;
    odomMsg.q[3] = - poseMsg.orientation.z;

    odomPub->publish(odomMsg);

    RCLCPP_INFO_ONCE(get_logger(), "Sent to PX4 as:");
    RCLCPP_INFO_ONCE(get_logger(), "P: %f, %f, %f", odomMsg.position[0],
                     odomMsg.position[1], odomMsg.position[2]);
    RCLCPP_INFO_ONCE(get_logger(), "q: %f, %f, %f, %f",
                     odomMsg.q[0], odomMsg.q[1], odomMsg.q[2], odomMsg.q[3]);
    break;
  }
}


int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MocapPX4Bridge>());
	rclcpp::shutdown();
	return 0;
}
