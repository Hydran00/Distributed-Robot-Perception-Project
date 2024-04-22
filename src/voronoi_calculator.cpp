// Example code demonstrating find_voronoi_cell function
//
// Author   : Chris H. Rycroft (LBL / UC Berkeley)
// Email    : chr@alum.mit.edu
// Date     : August 30th 2011

#include <unistd.h>

#include <iostream>

#include "project/utils.h"
#include "voro++.hh"
// ROS2
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
// tf2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <unistd.h>
using namespace std;
using namespace voro;
using namespace std::chrono_literals;

class VoronoiCalculator : public rclcpp::Node {
 public:
  VoronoiCalculator() : Node("voronoi_calculator") {
    // avoid storing the pointcloud if transform is not available
    rclcpp::sleep_for(2s);

    this->declare_parameter("prefix", "1_");
    prefix_ = this->get_parameter("prefix").as_string();

    tf_buffer_1_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_1_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_1_);

    tf_buffer_2_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_2_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_2_);

    container_ = std::make_shared<container>(-con_size, con_size, -con_size,
                                             con_size, -con_size, con_size, 5,
                                             5, 5, false, false, false, 8);
    timer_ = this->create_wall_timer(
        100ms, std::bind(&VoronoiCalculator::updateVoronoi, this));

    this->declare_parameter("target_frame", "world");
    target_frame_ = this->get_parameter("target_frame").as_string();

    this->declare_parameter("input_frame", "camera");
    input_frame_ = prefix_ + this->get_parameter("input_frame").as_string();

    this->declare_parameter("input_frame_other_robot", "camera");
    input_frame_other_robot_ =
        "2_" + this->get_parameter("input_frame_other_robot").as_string();

    this->declare_parameter("output_topic", "target_frame");
    output_topic_ = "/robot" + std::string(1, prefix_[0]) + "/" +
                    this->get_parameter("output_topic").as_string();

    // publisher
    target_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/robot" + std::string(1, prefix_[0]) + "/target_frame", 10);

    // print params
    RCLCPP_INFO(this->get_logger(), "target_frame: %s", target_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "input_frame: %s", input_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "input_frame_other_robot: %s",
                input_frame_other_robot_.c_str());
    RCLCPP_INFO(this->get_logger(), "output_topic: %s", output_topic_.c_str());
  }

 private:
  void updateVoronoi() {
    try {
      std::cout <<"input_frame: "<<input_frame_<<std::endl;
      std::cout <<"input_frame_other_robot: "<<input_frame_other_robot_<<std::endl;
      auto r1_pose_ = tf_buffer_1_->lookupTransform(target_frame_, input_frame_,
                                             tf2::TimePointZero);
      auto r2_pose_ = tf_buffer_2_->lookupTransform(
          target_frame_, input_frame_other_robot_, tf2::TimePointZero);

      std::cout << "r1_pose: \n"
                << r1_pose_.transform.translation.x << " "
                << r1_pose_.transform.translation.y << " "
                << r1_pose_.transform.translation.z << std::endl;
      std::cout << "r2_pose \n"
                << r2_pose_.transform.translation.x << " "
                << r2_pose_.transform.translation.y << " "
                << r2_pose_.transform.translation.z << std::endl;

      const double r1_x = r1_pose_.transform.translation.x;
      const double r1_y = r1_pose_.transform.translation.y;
      const double r1_z = r1_pose_.transform.translation.z;

      container_->clear();
      container_->put(0, r1_pose_.transform.translation.x,
                      r1_pose_.transform.translation.y,
                      r1_pose_.transform.translation.z);
      container_->put(1, r2_pose_.transform.translation.x,
                      r2_pose_.transform.translation.y,
                      r2_pose_.transform.translation.z);
      double x, y, z, rx, ry, rz;
      int j, current_cell_idx = 0;
      // retrieve the cell id of the considered robot
      bool success = container_->find_voronoi_cell(
          r1_pose_.transform.translation.x, r1_pose_.transform.translation.y,
          r1_pose_.transform.translation.z, rx, ry, rz, j);

      // extract vertices of the j cell
      double cell_vol;
      voro::c_loop_all cla(*container_);
      voro::voronoicell c;
      if (cla.start()) do {
          if (container_->compute_cell(c, cla)) {
            cla.pos(current_cell_idx, x, y, z, rx);
            if (current_cell_idx == j) {
              for (size_t i = 0; i < c.p; i++) {
                vertices.push_back(Eigen::Vector3d(
                    r1_x + 0.5 * c.pts[3 * i], r1_y + 0.5 * c.pts[3 * i + 1],
                    r1_z + 0.5 * c.pts[3 * i + 2]));
              }
            }
            cell_vol = c.volume();
            break;
          }
        } while (cla.inc());
      std::cout << "SIZE: " << vertices.size() << std::endl;
      if (vertices.size() > 0) {
        auto res = integrate_vector_valued_pdf_over_polyhedron(vertices,
                                                               container_, j);
        vertices.clear();
      }
    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "Could not get transform: %s",
                   ex.what());
      // return;
    }
    std::cout << "-------------------------" << std::endl;
  }
  // subscribers and publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  // get robot end effector pose
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_1_, tf_buffer_2_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_1_, tf_listener_2_;
  geometry_msgs::msg::TransformStamped transform_;
  // utils
  std::string topic_prefix_;
  geometry_msgs::msg::TransformStamped r1_pose_, r2_pose_;
  geometry_msgs::msg::PoseStamped pose_;
  std::vector<Eigen::Vector3d> vertices;
  bool cell_found_ = false;
  const double con_size = 2.0;
  const double cube_edge = 0.05;
  const double volume_cube = pow(cube_edge, 3);
  std::string input_frame_, input_frame_other_robot_;
  std::string output_topic_;
  std::string prefix_;
  std::string target_frame_;
  // voronoi
  std::shared_ptr<container> container_;
  // log
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VoronoiCalculator>());
  rclcpp::shutdown();
  return 0;
}