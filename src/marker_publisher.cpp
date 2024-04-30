
#include <iostream>

// ROS2
#include "geometry_msgs/msg/pose_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class MarkerPublisher : public rclcpp::Node {
 public:
  MarkerPublisher() : Node("marker_publisher") {
    // timer_(this->create_wall_timer(
    //     std::chrono::milliseconds(30),
    //     std::bind(&MarkerPublisher::publishMarker, this))) {
    // Initialize your vectors here
    // Initialize publishers here if needed
    markers_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "visualization_marker", 10);
  }
  visualization_msgs::msg::MarkerArray marker_array_;
  std::string prefix_1_ = "";
  std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>>
      markers_publisher_;

 private:
  void publishMarker() {
    marker_array_.markers.clear();
    visualization_msgs::msg::Marker mesh_msg;
    mesh_msg.header.frame_id = "world";
    mesh_msg.header.stamp = this->now();
    mesh_msg.ns = prefix_1_;
    mesh_msg.id = 0;
    mesh_msg.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    mesh_msg.action = visualization_msgs::msg::Marker::MODIFY;
    mesh_msg.scale.x = 1.0;
    mesh_msg.scale.y = 1.0;
    mesh_msg.scale.z = 1.0;
    mesh_msg.color.r = 1.0;
    mesh_msg.color.a = 1.0;

    // visualization_msgs::msg::Marker text_msg;
    // text_msg.header.frame_id = "world";
    // text_msg.header.stamp = this->now();
    // text_msg.ns = prefix_1_;
    // text_msg.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    // text_msg.action = visualization_msgs::msg::Marker::MODIFY;
    // text_msg.scale.z = 0.05;
    // text_msg.color.r = 1.0;
    // text_msg.color.g = 1.0;
    // text_msg.color.b = 1.0;
    // text_msg.color.a = 0.85;

    // visualization_msgs::msg::Marker arrow_msg;
    // arrow_msg.header.frame_id = "world";
    // arrow_msg.header.stamp = this->now();
    // arrow_msg.ns = prefix_1_;
    // arrow_msg.type = visualization_msgs::msg::Marker::ARROW;
    // arrow_msg.action = visualization_msgs::msg::Marker::MODIFY;
    // arrow_msg.scale.x = 0.02;
    // // arrow_msg.scale.y = 0.1;
    // arrow_msg.scale.z = 0.03;
    // arrow_msg.color.r = 1.0;
    // arrow_msg.color.g = 0.0;
    // arrow_msg.color.b = 0.0;
    // arrow_msg.color.a = 1.0;

    // std::vector<visualization_msgs::msg::Marker> texts;
    // std::vector<visualization_msgs::msg::Marker> arrows;
    // // generate colormap
    // std::vector<double> coloredArray =
    //     generateBluesColormap((int)pdf_coeffs_.size(), pdf_coeffs_);

    // geometry_msgs::msg::Point p;
    // geometry_msgs::msg::Point normal;
    // std_msgs::msg::ColorRGBA color;

    // for (size_t i = 0; i < faces_vertices_.size(); i++) {
    //   auto vertices = faces_vertices_[i];
    //   for (size_t j = 0; j < vertices.size(); j++) {
    //     p.x = vertices[j](0) / 8;
    //     p.y = vertices[j](1) / 8;
    //     p.z = vertices[j](2) / 8;
    //     mesh_msg.points.push_back(p);
    //   }
    //   color.r = coloredArray[i];
    //   color.g = coloredArray[i];
    //   color.b = coloredArray[i];
    //   color.a = 1.0;
    //   mesh_msg.colors.push_back(color);
    //   text_msg.pose.position.x = face_centers_[i](0) / 2;
    //   text_msg.pose.position.y = face_centers_[i](1) / 2;
    //   text_msg.pose.position.z = face_centers_[i](2) / 2;
    //   text_msg.text = std::to_string(i);
    //   text_msg.id = i + 1;
    //   texts.push_back(text_msg);
    //   arrow_msg.points.clear();
    //   normal.x = 0.0;
    //   normal.y = 0.0;
    //   normal.z = 0.0;
    //   arrow_msg.points.push_back(normal);
    //   normal.x = face_normals_[i](0);
    //   normal.y = face_normals_[i](1);
    //   normal.z = face_normals_[i](2);
    //   arrow_msg.points.push_back(normal);
    //   arrow_msg.id = (int)faces_vertices_.size() + i + 1;
    //   arrows.push_back(arrow_msg);
    // }
    // // CURRENT ORIENTATION
    // arrow_msg.points.clear();
    // normal.x = 0.0;
    // normal.y = 0.0;
    // normal.z = 0.0;
    // arrow_msg.points.push_back(normal);
    // normal.x = current_axis_(0);
    // normal.y = current_axis_(1);
    // normal.z = current_axis_(2);
    // arrow_msg.points.push_back(normal);
    // arrow_msg.id = 2 * (int)faces_vertices_.size() + 1;
    // arrow_msg.color.g = 1.0;
    // arrow_msg.color.r = 0.0;

    // // std::cout << "/////////////////" << std::endl;
    marker_array_.markers.push_back(mesh_msg);
    // for (auto text : texts) {
    //   marker_array_.markers.push_back(text);
    // }
    // // for (auto arrow : arrows) {
    // //   marker_array_.markers.push_back(arrow);
    // // }
    // marker_array_.markers.push_back(arrow_msg);
    markers_publisher_->publish(marker_array_);
  }
  rclcpp::TimerBase::SharedPtr timer_;
};

// int main(int argc, char **argv) {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<MarkerPublisher>());
//   rclcpp::shutdown();
//   return 0;
// }
