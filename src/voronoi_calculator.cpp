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
#include "std_msgs/msg/float64_multi_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

// tf2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "tf2/convert.h"

#define N 20

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

    container_pdf_ = std::make_unique<container>(
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
    pdf_coeffs_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/pdf_coeffs" + prefix_1_, 1);
    pdf_coeffs_sub_ =
        this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/pdf_coeffs" + prefix_2_, 1,
            std::bind(&VoronoiCalculator::pdfCoeffsCallback, this,
                      std::placeholders::_1));
    markers_publisher_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "markers" + prefix_1_, 1);

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

    init_icosahedron_planes(planes_, 2.0);
    for (auto plane : planes_) {
      container_->add_wall(*plane);
    }
    // draw this voronoi
    // prepare outer polyhedron
    // compute_subdivided_polyhedron(container_pdf_, 1.0);

    // ========================================================
    double scale = 2.0;
    std::vector<std::shared_ptr<voro::wall_plane>> planes;
    container_pdf_->clear();
    init_icosahedron_planes(planes, scale);
    for (auto plane : planes) {
      container_pdf_->add_wall(*plane);
    }
    container_pdf_->put(0, 0, 0, 0);

    // print container
    voro::c_loop_all clp(*container_pdf_);
    voro::voronoicell c_pdf;
    if (clp.start()) do {
        container_pdf_->compute_cell(c_pdf, clp);
      } while (clp.inc());
    std::map<int, std::vector<int>> vertices_indeces;
    std::vector<int> tmp;
    c_pdf.face_vertices(tmp);

    // //populate map

    int length = 0;
    int j = 0, k = 0;
    for (int i = 0; i < tmp.size(); i += length + 1) {
      length = tmp[i];
      if (length == 0) {
        break;
      }
      for (int j = i + 1; j < i + 1 + length; j++) {
        vertices_indeces[k].push_back(tmp[j]);
      }
      k++;
    }

    int i = 0;

    std::vector<Eigen::Vector3d> vertices;
    std::vector<double> tmp_v;

    c_pdf.vertices(tmp_v);

    for (int i = 0; i < tmp_v.size(); i += 3) {
      vertices.push_back(Eigen::Vector3d(tmp_v[i], tmp_v[i + 1], tmp_v[i + 2]));
    }

    // loop face indices
    for (auto const &face : vertices_indeces) {
      std::vector<Eigen::Vector3d> face_vertices;
      for (auto const &vertex_index : face.second) {
        face_vertices.push_back(vertices[vertex_index]);
      }
      faces_vertices_[face.first] = face_vertices;
    }

    int z = 0;
    container_pdf_->clear();
    for (auto const &face : faces_vertices_) {
      Eigen::Vector3d center = 0.5 * compute_center(face.second);
      if (container_pdf_->point_inside(center.x(), center.y(), center.z())) {
        container_pdf_->put(z, center.x(), center.y(), center.z());
        centers_.push_back(center);
      }
      z++;
    }
    // c_loop_all clp2(*container_pdf_);
    // voronoicell c_pdf2;
    // i = 0;

    // compute normals given centers
    for (auto center : centers_) {
      // consider vector from center to origin
      Eigen::Vector3d normal = center;
      normal.normalize();
      std::cout << "Norm is " << normal.norm() << std::endl;
      normals_.push_back(normal);
    }
    // INIT PDF
    container_->put(0, 0, 0, 0);
    container_->compute_all_cells();
    // container_pdf_->put(0, 0, 0, 0);
    // container_pdf_->compute_all_cells();
    container_->draw_cells_gnuplot("starting_poly.gnu");
    container_pdf_->draw_cells_gnuplot("pdf_polyhedron.gnu");
    for (size_t i = 0; i < N; i++) {
      pdf_coeffs_.push_back(0.0);
    }
    std::cout << "Node started correctly" << std::endl;
  }

 private:
  void pdfCoeffsCallback(
      const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    for (size_t i = 0; i < msg->data.size(); i++) {
      pdf_coeffs_[i] = std::max(msg->data[i], pdf_coeffs_[i]);
      // pdf_coeffs_[i] = pdf_coeffs_[i];
    }
  }
  void updateVoronoi() {
    try {
      std::cout << "Input frame: " << input_frame_ << std::endl;
      std::cout << "voronoi frame: " << voronoi_frame_ << std::endl;

      auto r1_pose_ = tf_buffer_1_->lookupTransform(
          voronoi_frame_, input_frame_, tf2::TimePointZero, 10ms);
      auto r2_pose_ = tf_buffer_2_->lookupTransform(
          voronoi_frame_, input_frame_other_robot_, tf2::TimePointZero, 10ms);

      std::cout << "real quat: \n"
                << r1_pose_.transform.rotation.x << " "
                << r1_pose_.transform.rotation.y << " "
                << r1_pose_.transform.rotation.z
                << r1_pose_.transform.rotation.w << std::endl;
      Eigen::AngleAxisd angle_axis(Eigen::Quaterniond(
          r1_pose_.transform.rotation.w, r1_pose_.transform.rotation.x,
          r1_pose_.transform.rotation.y, r1_pose_.transform.rotation.z));

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

      // extract

      if (vertices.size() > 0) {
        if (debug_) {
          publish_voronoi_vertices(vertices);
        }
        Eigen::Quaterniond quat(
            r1_pose_.transform.rotation.w, r1_pose_.transform.rotation.x,
            r1_pose_.transform.rotation.y, r1_pose_.transform.rotation.z);
        auto rotmat = quat.toRotationMatrix();
        // print third column
        std::cout << "rotmat: " << rotmat.col(2).transpose() << std::endl;
        // compute the pdf coefficient
        for (size_t i = 0; i < N; i++) {
          pdf_coeffs_[i] = std::max(
              angleBetweenNormals(normals_[i], rotmat.col(2).normalized()) /
                  2.0,
              pdf_coeffs_[i]);
        }
        // publish the pdf coefficients
        std_msgs::msg::Float64MultiArray msg;
        msg.data = pdf_coeffs_;
        for (size_t i = 0; i < N; i++) {
          std::cout << "pdf_coeffs_[" << i << "]: " << pdf_coeffs_[i]
                    << std::endl;
        }
        std::cout << "====================" << std::endl;
        pdf_coeffs_pub_->publish(msg);
        publishTriangleList();

        auto res = integrate_vector_valued_pdf_over_polyhedron(
            vertices, container_, container_pdf_, j, pdf_coeffs_);
        // publish the result
        // project onto sphere surface
        std::cout << " x: " << res(0) << " y: " << res(1) << " z: " << res(2)
                  << std::endl;
        res = projectOnSphere(res, 0.7);

        pose_.header.stamp = this->now();
        pose_.header.frame_id = voronoi_frame_;
        pose_.pose.position.x = res(0);  // r1_x;
        pose_.pose.position.y = res(1);  // r1_y;
        pose_.pose.position.z = res(2);  // r1_z;

        auto trans = tf_buffer_1_->lookupTransform(base_frame_, voronoi_frame_,
                                                   tf2::TimePointZero);
        // apply orientation so that the robot is always facing the center
        // (0,0,0) create quaternion from axis angle with no rotation

        // std::cout << " x: " << r1_x << "\n y: " << r1_y << "\n z: " << r1_z
        //           << std::endl;

        // Eigen::AngleAxisd angle_axis(M_PI, Eigen::Vector3d(0, 0,
        // 1).normalized()); Eigen::Quaterniond q(angle_axis);
        auto q = computeQuaternion(
            Eigen::Vector3d(pose_.pose.position.x, pose_.pose.position.y,
                            pose_.pose.position.z),
            Eigen::Vector3d(0, 0, 0));
        pose_.pose.orientation.x = q.x();
        pose_.pose.orientation.y = q.y();
        pose_.pose.orientation.z = q.z();
        pose_.pose.orientation.w = q.w();

        // std::cout << " AA: " << angle_axis.angle() << " "<<
        // angle_axis.axis().transpose() << std::endl;

        tf2::doTransform(pose_, pose_, trans);
        // TODO check that the pose is outside of the sphere
        auto new_pose =
            Eigen::Vector3d(pose_.pose.position.x, pose_.pose.position.y,
                            pose_.pose.position.z);
        new_pose = projectOnSphere(new_pose, 0.6);
        pose_.pose.position.x = new_pose(0);
        pose_.pose.position.y = new_pose(1);
        pose_.pose.position.z = new_pose(2);
        target_pub_->publish(pose_);
        if (vertices.size() > 0) {
          vertices.clear();
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
  void publishTriangleList() {
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

    visualization_msgs::msg::Marker text_msg;
    text_msg.header.frame_id = "world";
    text_msg.header.stamp = this->now();
    text_msg.ns = prefix_1_;
    text_msg.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_msg.action = visualization_msgs::msg::Marker::MODIFY;
    text_msg.scale.z = 0.05;
    text_msg.color.r = 1.0;
    text_msg.color.g = 1.0;
    text_msg.color.b = 1.0;
    text_msg.color.a = 0.85;

    visualization_msgs::msg::Marker arrow_msg;
    arrow_msg.header.frame_id = "world";
    arrow_msg.header.stamp = this->now();
    arrow_msg.ns = prefix_1_;
    arrow_msg.type = visualization_msgs::msg::Marker::ARROW;
    arrow_msg.action = visualization_msgs::msg::Marker::MODIFY;
    arrow_msg.scale.x = 0.02;
    // arrow_msg.scale.y = 0.1;
    arrow_msg.scale.z = 0.03;
    arrow_msg.color.r = 1.0;
    arrow_msg.color.g = 0.0;
    arrow_msg.color.b = 0.0;
    arrow_msg.color.a = 1.0;

    std::vector<visualization_msgs::msg::Marker> texts;
    std::vector<visualization_msgs::msg::Marker> arrows;
    // generate colormap
    std::vector<double> coloredArray =
        generateBluesColormap((int)pdf_coeffs_.size(), pdf_coeffs_);

    geometry_msgs::msg::Point p;
    geometry_msgs::msg::Point normal;
    std_msgs::msg::ColorRGBA color;

    for (int i = 0; i < N; i++) {
      auto vertices = faces_vertices_[i];
      for (size_t j = 0; j < vertices.size(); j++) {
        p.x = vertices[j](0) / 6;
        p.y = vertices[j](1) / 6;
        p.z = vertices[j](2) / 6;
        mesh_msg.points.push_back(p);
      }
      color.r = 0.0;
      color.g = 0.0;
      color.b = coloredArray[i];
      color.a = 1.0;
      mesh_msg.colors.push_back(color);
      text_msg.pose.position.x = centers_[i](0) / 2;
      text_msg.pose.position.y = centers_[i](1) / 2;
      text_msg.pose.position.z = centers_[i](2) / 2;
      text_msg.text = std::to_string(i);
      text_msg.id = i + 1;
      texts.push_back(text_msg);
      arrow_msg.points.clear();
      normal.x = 0.0;
      normal.y = 0.0;
      normal.z = 0.0;
      arrow_msg.points.push_back(normal);
      normal.x = normals_[i](0);
      normal.y = normals_[i](1);
      normal.z = normals_[i](2);
      arrow_msg.points.push_back(normal);
      arrow_msg.id = N + i + 1;
      arrows.push_back(arrow_msg);
    }
    // CURRENT ORIENTATION
    arrow_msg.points.clear();
    normal.x = 0.0;
    normal.y = 0.0;
    normal.z = 0.0;
    arrow_msg.points.push_back(normal);
    normal.x = current_axis_(0);
    normal.y = current_axis_(1);
    normal.z = current_axis_(2);
    arrow_msg.points.push_back(normal);
    arrow_msg.id = 2 * N + 1;
    arrow_msg.color.g = 1.0;
    arrow_msg.color.r = 0.0;

    std::cout << "/////////////////" << std::endl;
    marker_array_.markers.push_back(mesh_msg);
    for (auto text : texts) {
      marker_array_.markers.push_back(text);
    }
    for (auto arrow : arrows) {
      marker_array_.markers.push_back(arrow);
    }
    marker_array_.markers.push_back(arrow_msg);
    markers_publisher_->publish(marker_array_);
  }

  // subscribers and publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr
      voronoi_vertices_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      pdf_coeffs_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
      pdf_coeffs_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      markers_publisher_;
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
  visualization_msgs::msg::MarkerArray marker_array_;
  Eigen::Vector3d current_axis_;
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
  std::shared_ptr<voro::container> container_;
  std::shared_ptr<voro::container> container_pdf_;
  std::vector<std::shared_ptr<voro::wall_plane>> planes_;
  std::shared_ptr<voro::wall_sphere> sphere;
  std::vector<Eigen::Vector3d> centers_;
  std::vector<Eigen::Vector3d> normals_;
  std::map<int, std::vector<Eigen::Vector3d>> faces_vertices_;
  std::vector<double> pdf_coeffs_;
  // log
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VoronoiCalculator>());
  rclcpp::shutdown();
  return 0;
}