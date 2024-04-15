// Example code demonstrating find_voronoi_cell function
//
// Author   : Chris H. Rycroft (LBL / UC Berkeley)
// Email    : chr@alum.mit.edu
// Date     : August 30th 2011

#include <iostream>
#include <unistd.h>
#include "voro++.hh"
// ROS2
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
// tf2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace std;
using namespace voro;
using namespace std::chrono_literals;

class VoronoiCalculator : public rclcpp::Node
{
public:
  VoronoiCalculator() : Node("voronoi_calculator")
  {
    // avoid storing the pointcloud if transform is not available
    rclcpp::sleep_for(2s);

    this->declare_parameter("topic_prefix", "voronoi_vertices");
    topic_prefix_ = this->get_parameter("topic_prefix").as_string();


    tf_buffer_1_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_1_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_1_);
    tf_buffer_2_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_2_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_2_);

    container_ = std::make_shared<container>(-1, 1, -1, 1, -1, 1, 5, 5, 5, false, false, false, 8);
    timer_ = this->create_wall_timer(20ms, std::bind(&VoronoiCalculator::updateVoronoi, this));

    this->declare_parameter("target_frame", "world");
    target_frame_ = this->get_parameter("target_frame").as_string();

    this->declare_parameter("input_frame_1", "1_camera");
    input_frame_1_ = this->get_parameter("input_frame_1").as_string();

    this->declare_parameter("input_frame_2", "2_camera");
    input_frame_2_ = this->get_parameter("input_frame_2").as_string();

    this->declare_parameter("output_topic_1", "voronoi_vertices_1");
    output_topic_1_ = this->get_parameter("output_topic_1").as_string();

    this->declare_parameter("output_topic_2", "voronoi_vertices_2");
    output_topic_2_ = this->get_parameter("output_topic_2").as_string();
    
    // publisher
    voronoi_publisher_1_ = this->create_publisher<geometry_msgs::msg::PoseArray>(output_topic_1_, 1);
    voronoi_publisher_2_ = this->create_publisher<geometry_msgs::msg::PoseArray>(output_topic_2_, 1);

    RCLCPP_INFO(this->get_logger(), "Voronoi Calculator has been initialized");
    RCLCPP_INFO(this->get_logger(), "output_topic_1: %s", output_topic_1_.c_str());
    RCLCPP_INFO(this->get_logger(), "output_topic_2: %s", output_topic_2_.c_str());
    RCLCPP_INFO(this->get_logger(), "target_frame: %s", target_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "input_frame_1: %s", input_frame_1_.c_str());
    RCLCPP_INFO(this->get_logger(), "input_frame_2: %s", input_frame_2_.c_str());

  }

private:
  void callback1(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
  }
  void callback2(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
  }

  void updateVoronoi()
  {
    try
    {
      r1_pose_ = tf_buffer_1_->lookupTransform(target_frame_, input_frame_1_, tf2::TimePointZero);
      r2_pose_ = tf_buffer_2_->lookupTransform(target_frame_, input_frame_2_, tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex)
    {
      // RCLCPP_ERROR(this->get_logger(), "Could not get transform: %s", ex.what());
      return;
    }

    container_->clear();
    container_->put(0, r1_pose_.transform.translation.x, r1_pose_.transform.translation.y, r1_pose_.transform.translation.z);
    container_->put(1, r2_pose_.transform.translation.x, r2_pose_.transform.translation.y, r2_pose_.transform.translation.z);
    container_->draw_particles("robots.gnu");
    c_loop_all cla(*container_);
    voronoicell c;
    double x, y, z, r1;
    int j;
    f1 = safe_fopen("find_voro_cell.vol", "w");
    f2 = safe_fopen("find_voro_cell_v.gnu", "w");
    if (cla.start())
    {
      do
      {
        if (container_->compute_cell(c, cla))
        {
          cell_found_ = true;
          cla.pos(j, x, y, z, r1);
          c.vertices(x, y, z, vertices);
          for (int i = 0; i < 3 * c.p; i += 3)
          {
            pose_.position.x = vertices[i];
            pose_.position.y = vertices[i + 1];
            pose_.position.z = vertices[i + 2];
            if (j == 0)
            {
              voronoi_1_.poses.push_back(pose_);
            }
            else
            {
              voronoi_2_.poses.push_back(pose_);
            }
          }
          c.draw_gnuplot(x, y, z, f2);
        }
      } while (cla.inc());
      if (cell_found_)
      {
        voronoi_1_.header.stamp = this->now();
        voronoi_1_.header.frame_id = target_frame_;
        voronoi_publisher_1_->publish(voronoi_1_);
        voronoi_2_.header.stamp = this->now();
        voronoi_2_.header.frame_id = target_frame_;
        voronoi_publisher_2_->publish(voronoi_2_);
        voronoi_1_.poses.clear();
        voronoi_2_.poses.clear();
      }
    }
    cell_found_ = false;
    fclose(f1);
    fclose(f2);
  }
  // subscribers and publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr voronoi_publisher_1_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr voronoi_publisher_2_;
  rclcpp::TimerBase::SharedPtr timer_;
  // get robot end effector pose
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_1_, tf_buffer_2_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_1_, tf_listener_2_;
  geometry_msgs::msg::TransformStamped transform_1_, transform_2_;
  // utils
  std::string topic_prefix_;
  geometry_msgs::msg::TransformStamped r1_pose_, r2_pose_;
  geometry_msgs::msg::PoseArray voronoi_1_, voronoi_2_;
  geometry_msgs::msg::Pose pose_;
  std::vector<double> vertices = {0.0, 0.0, 0.0};
  bool cell_found_=false;
  std::string input_frame_1_, input_frame_2_, target_frame_, output_topic_1_, output_topic_2_;
  // voronoi
  std::shared_ptr<container> container_;
  // log
  FILE *f1;
  FILE *f2;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VoronoiCalculator>());
  rclcpp::shutdown();
  return 0;
}