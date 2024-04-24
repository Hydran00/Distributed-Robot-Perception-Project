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
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
// tf2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <unistd.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "tf2/convert.h"
using namespace std;
using namespace voro;
using namespace std::chrono_literals;

class VoronoiCalculator : public rclcpp::Node {
 public:
  VoronoiCalculator() : Node("voronoi_calculator") {
    // avoid storing the pointcloud if transform is not available
    rclcpp::sleep_for(2s);

    this->declare_parameter("prefix_1", "");
    prefix_1_ = this->get_parameter("prefix_1").as_string();

    this->declare_parameter("prefix_2", "");
    prefix_2_ = this->get_parameter("prefix_2").as_string();

    this->declare_parameter("debug", false);
    debug_ = this->get_parameter("debug").as_bool();

    this->declare_parameter("voronoi_frame", "world");
    voronoi_frame_ = this->get_parameter("voronoi_frame").as_string();

    this->declare_parameter("input_frame", "camera");
    input_frame_ = prefix_1_ + this->get_parameter("input_frame").as_string();

    this->declare_parameter("input_frame_other_robot", "camera");
    input_frame_other_robot_ =
        prefix_2_ + this->get_parameter("input_frame_other_robot").as_string();

    this->declare_parameter("target_topic", "target_frame");
    target_topic_ = "/robot" + prefix_1_ + "/" +
                    this->get_parameter("target_topic").as_string();

    this->declare_parameter("base_frame", "base_link");
    base_frame_ = prefix_1_ + this->get_parameter("base_frame").as_string();

    tf_buffer_1_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_1_ =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_1_);

    tf_buffer_2_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_2_ =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_2_);

    container_ = std::make_shared<container>(
        con_size_xmin, con_size_xmax, con_size_ymin, con_size_ymax,
        con_size_zmin, con_size_zmax, 5, 5, 5, false, false, false, 8);
    container_pdf_ = std::make_shared<container>(
        con_size_xmin, con_size_xmax, con_size_ymin, con_size_ymax,
        con_size_zmin, con_size_zmax, 5, 5, 5, false, false, false, 8);

    timer_ = this->create_wall_timer(
        20ms, std::bind(&VoronoiCalculator::updateVoronoi, this));

    // publisher
    target_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        target_topic_, 1);
    if (debug_) {
      voronoi_vertices_pub_ =
          this->create_publisher<geometry_msgs::msg::PoseArray>(
              "/voronoi_vertices" + prefix_1_, 1);
    }
    // print params
    RCLCPP_INFO(this->get_logger(), "DEBUG FLAG: %d", debug_);
    RCLCPP_INFO(this->get_logger(), "voronoi_frame: %s",
                voronoi_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "input_frame: %s", input_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "input_frame_other_robot: %s",
                input_frame_other_robot_.c_str());
    RCLCPP_INFO(this->get_logger(), "base_frame: %s", base_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "target_topic: %s", target_topic_.c_str());

    // sphere = std::make_shared<wall_sphere>(0, 0, 0, 1.0);
    // create_icosahedron(container_, 1);

    init_icosahedron_planes(planes_, 1);
    for (auto plane : planes_) {
      container_->add_wall(*plane);
    }
    for (auto plane : planes_) {
      container_pdf_->add_wall(*plane);
    }
    container_pdf_->import("sphere_points.dat");
    std::cout << "Voronoi calculator initialized" << std::endl;
  }

 private:
  void updateVoronoi() {
    try {
      auto r1_pose_ = tf_buffer_1_->lookupTransform(
          voronoi_frame_, input_frame_, tf2::TimePointZero, 10ms);
      auto r2_pose_ = tf_buffer_2_->lookupTransform(
          voronoi_frame_, input_frame_other_robot_, tf2::TimePointZero, 10ms);

      const double r1_x = r1_pose_.transform.translation.x;
      const double r1_y = r1_pose_.transform.translation.y;
      const double r1_z = r1_pose_.transform.translation.z;
      const double r2_x = r2_pose_.transform.translation.x;
      const double r2_y = r2_pose_.transform.translation.y;
      const double r2_z = r2_pose_.transform.translation.z;
      container_->clear();

      if (container_->point_inside(r1_x, r1_y, r1_z)) {
        container_->put(0, r1_pose_.transform.translation.x,
                        r1_pose_.transform.translation.y,
                        r1_pose_.transform.translation.z);
      }
      if (container_->point_inside(r2_x, r2_y, r2_z)) {
        container_->put(1, r2_pose_.transform.translation.x,
                        r2_pose_.transform.translation.y,
                        r2_pose_.transform.translation.z);
      }
      container_pdf_->clear();
      container_pdf_->put(0, 0, 0, 0);
      double x, y, z, rx, ry, rz;
      int j, current_cell_idx = 0;

      // retrieve the cell id of the considered robot
      bool success = container_->find_voronoi_cell(
          r1_pose_.transform.translation.x, r1_pose_.transform.translation.y,
          r1_pose_.transform.translation.z, rx, ry, rz, j);
      // container_->add_wall(*sphere);
      // Place particles to create a voronoi sphere
      // container_->import("sphere_points.dat");

      // load particles from file

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
              break;
            }
          }
        } while (cla.inc());
      if (vertices.size() > 0) {
        if (debug_) {
          publish_voronoi_vertices(vertices);
        }
        auto res = integrate_vector_valued_pdf_over_polyhedron(vertices,
                                                               container_, j);
        // publish the result
        pose_.header.stamp = this->now();
        pose_.header.frame_id = voronoi_frame_;
        pose_.pose.position.x = res(0);
        pose_.pose.position.y = res(1);
        pose_.pose.position.z = res(2);
        auto trans = tf_buffer_1_->lookupTransform(base_frame_, voronoi_frame_,
                                                   tf2::TimePointZero);
        tf2::doTransform(pose_, pose_, trans);
        // apply orientation
        pose_.pose.orientation.x = -0.707;
        pose_.pose.orientation.y = 0.0;
        pose_.pose.orientation.z = 0.0;
        pose_.pose.orientation.w = 0.707;

        // target_pub_->publish(pose_);
        if (vertices.size() > 0) {
          vertices.clear();
        }
        if (debug_) {
          // print container
          FILE *f1 = safe_fopen((prefix_1_ + "PART.gnu").c_str(), "w");
          // manually store particles
          // fprintf(f1, "%d %g %g %g\n", 0, r1_x, r1_y, r1_z);
          // fprintf(f1, "%d %g %g %g\n", 1, r2_x, r2_y, r2_z);
          fclose(f1);
          container_->draw_cells_gnuplot((prefix_1_ + "voronoi.gnu").c_str());
          container_pdf_->draw_cells_gnuplot(
              (prefix_1_ + "voronoi_pdf.gnu").c_str());
          c_loop_all clp(*container_pdf_);
          voronoicell c_pdf;
          if (clp.start()) do {
              if (container_pdf_->compute_cell(c_pdf, clp)) {
                clp.pos(j, x, y, z, rx);
                // c_pdf.output_face_vertices();
                // v.output_vertices()
                // extract vertices indices

                std::map<int, std::vector<int>> vertices_indeces;
                std::vector<int> tmp;
                c_pdf.face_vertices(tmp);
                // print tmp
                for(int i = 0; i < tmp.size(); i++){
                  std::cout << tmp[i] << " ";
                }

                //populate map
                int j = 0;
                for(int i = 0; i < tmp.size(); i++){
                    if(tmp[i] < 1){
                      j++;
                  }else{
                    vertices_indeces[j].push_back(tmp[i]);
                  }
                }
                std::cout << "/////" <<std::endl;
                //print map   
                for (auto const& x : vertices_indeces)
                {
                    std::cout << x.first << ": ";
                    for (auto const& y : x.second)
                    {
                        std::cout << y << " ";
                    }
                    std::cout << std::endl;
                }

                //print vertices_indices
                // for (auto vertex_index : vertices_indeces) {
                //   std::cout << vertex_index << std::endl;
                // }
                // std::vector<Eigen::Vector3d> pts;
                // for (double *ptsp = pts + 4; ptsp < pts + (p << 2); ptsp += 4) {
                //   //  c->ptsp*0.5,c->ptsp[1]*0.5,c->ptsp[2]*0.5);
                //   pts.push_back(c->ptsp * 0.5, c->ptsp[1] * 0.5,
                //                 c->ptsp[2] * 0.5);
                // }
                // // compute vertices for each face
                // std::vector<std::vector<Eigen::Vector3d>> face_vertices;
                // for (auto vertex_index : vertices_indeces) {
                //   face_vertices.push_back(pts[vertex_index]);
                // }
                // extract vertices coordinates

                std::cout << std::endl;
              }
            } while (clp.inc());
        }
      }

    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "Could not get transform: %s",
                   ex.what());
      // return;
    }
  }
  void publish_voronoi_vertices(std::vector<Eigen::Vector3d> vertices) {
    // publish voronoi vertices
    voronoi_vertices_.header.stamp = this->now();
    voronoi_vertices_.header.frame_id = voronoi_frame_;
    voronoi_vertices_.poses.clear();
    for (const auto &vertex : vertices) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = vertex(0);
      pose.position.y = vertex(1);
      pose.position.z = vertex(2);
      voronoi_vertices_.poses.push_back(pose);
    }

    voronoi_vertices_pub_->publish(voronoi_vertices_);
  }

  // subscribers and publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr
      voronoi_vertices_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  // get robot end effector pose
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_1_, tf_buffer_2_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_1_, tf_listener_2_;
  geometry_msgs::msg::TransformStamped transform_;
  // utils
  geometry_msgs::msg::TransformStamped r1_pose_, r2_pose_;
  geometry_msgs::msg::PoseArray voronoi_vertices_;
  geometry_msgs::msg::PoseStamped pose_;
  std::vector<Eigen::Vector3d> vertices;
  bool cell_found_ = false;
  const double con_size_xmin = -2.0, con_size_xmax = 2.0;
  const double con_size_ymin = -2.0, con_size_ymax = 2.0;
  const double con_size_zmin = -2.0, con_size_zmax = 2.0;
  std::string input_frame_, input_frame_other_robot_;
  std::string prefix_1_, prefix_2_;
  std::string voronoi_frame_;
  std::string base_frame_;
  std::string target_topic_;
  bool debug_;
  // voronoi
  std::shared_ptr<container> container_, container_pdf_;
  std::vector<std::shared_ptr<voro::wall_plane>> planes_;
  std::shared_ptr<voro::wall_sphere> sphere;
  // log
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VoronoiCalculator>());
  rclcpp::shutdown();
  return 0;
}