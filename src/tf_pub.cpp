#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"


class FramePublisher : public rclcpp::Node
{
public:
  FramePublisher()
  : Node("tf_pub")
  {
    // Declare and acquire `turtlename` parameter
    prefix_ = this->declare_parameter<std::string>("prefix", "");
    prefix_  = this->get_parameter("prefix").as_string();

    // Initialize the transform broadcaster
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "robot"+prefix_+"pose", 1,
      std::bind(&FramePublisher::publish, this, std::placeholders::_1));
  }

private:
  void publish(const std::shared_ptr<geometry_msgs::msg::PoseStamped> msg)
  {
    geometry_msgs::msg::TransformStamped t;

    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = msg->header.stamp;
    t.header.frame_id = "world";
    t.child_frame_id = prefix_+"base_link";

    t.transform.translation.x = msg->pose.position.x;
    t.transform.translation.y = msg->pose.position.y;
    t.transform.translation.z = msg->pose.position.z;
    t.transform.rotation.x = msg->pose.orientation.x;
    t.transform.rotation.y = msg->pose.orientation.y;
    t.transform.rotation.z = msg->pose.orientation.z;
    t.transform.rotation.w = msg->pose.orientation.w;

    // Send the transformation
    tf_broadcaster_->sendTransform(t);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::string prefix_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FramePublisher>());
  rclcpp::shutdown();
  return 0;
}