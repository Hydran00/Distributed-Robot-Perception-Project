// standard headers
#include <iostream>
#include <string>
// package headers
#include "project/icosahedron.h"
#include "project/utils.h"
#include "voro++.hh"
// ROS2 headers
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
// PCL ROS headers
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
// tf2 headers
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2/convert.h"

using namespace voro;
using namespace std::chrono_literals;

class VoronoiCalculator : public rclcpp::Node {
 public:
  VoronoiCalculator() : Node("voronoi_calculator") {
    // avoid storing the pointcloud if transform is not available

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

    this->declare_parameter("output_frame", "base_link");
    output_frame_ = this->get_parameter("output_frame").as_string();

    this->declare_parameter("center_x", 0.0);
    center_x_ = this->get_parameter("center_x").as_double();

    this->declare_parameter("center_y", 0.0);
    center_y_ = this->get_parameter("center_y").as_double();

    this->declare_parameter("center_z", 0.0);
    center_z_ = this->get_parameter("center_z").as_double();

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
    mean_min_dist_pub_ =
        this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/mean_min_dist" + prefix_1_, 1);

    mean_min_dist_sub_ =
        this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/mean_min_dist" + prefix_2_, 1,
            std::bind(&VoronoiCalculator::meanMinDistCoeffsCallback, this,
                      std::placeholders::_1));

    point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/total_cloud", 1,
        std::bind(&VoronoiCalculator::pointCloudCallback, this,
                  std::placeholders::_1));
    cloud_ =
        pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
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

    // DEBUG
    for (int i=0; i<20; i++) {
      prev_multipliers.push_back(0);
    }

    utils::initIcosahedronPlanes(planes_, 2.0);
    for (auto plane : planes_) {
      container_->add_wall(*plane);
    }

    getIcosahedronFaceVertices(con_size_xmin, con_size_xmax, con_size_ymin,
                               con_size_ymax, con_size_zmin, con_size_zmax,
                               center_x_, center_y_, center_z_, faces_vertices_,
                               faces_centers_, faces_normals_, container_pdf_,
                               planes_pdf_);

    for (int i = 0; i < faces_vertices_.size(); i++) {
      mean_dist_with_nearest_.push_back(1.0);
    }
    // initialize pdf coefficients
    for (size_t i = 0; i < faces_vertices_.size(); i++) {
      pdf_coeffs_.push_back(0.0);
    }
    // Init robot velocity
    // auto r1_pose = tf_buffer_1_->lookupTransform(voronoi_frame_,
    // input_frame_,
    //                                              tf2::TimePointZero, 10ms);
    r1_last_pose_ = Eigen::Vector3d(0, 0, 0);
    r1_trans_velocity_ = Eigen::Vector3d(0, 0, 0);
    last_time_ = this->now();
    annealing_start_time_ = this->now();
    //sleep 1s
    rclcpp::sleep_for(1s);
    std::cout << "Node started correctly" << std::endl;
  }

 private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    pcl::fromROSMsg(*msg, *cloud_);
  }
  
  void pdfCoeffsCallback(
      const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    for (size_t i = 0; i < msg->data.size(); i++) {
      pdf_coeffs_[i] = std::max(msg->data[i], pdf_coeffs_[i]);
    }
  }
  
  void meanMinDistCoeffsCallback(
      const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    for (size_t i = 0; i < msg->data.size(); i++) {
      mean_dist_with_nearest_[i] =
          std::min(msg->data[i], mean_dist_with_nearest_[i]);
    }
  }

  void updateVoronoi() {
    if (cloud_->size() == 0) {
      return;
    }

    try {
      r1_pose_ = tf_buffer_1_->lookupTransform(voronoi_frame_, input_frame_,
                                               tf2::TimePointZero, 10ms);
      r2_pose_ = tf_buffer_2_->lookupTransform(
          voronoi_frame_, input_frame_other_robot_, tf2::TimePointZero, 10ms);

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

      if (debug_) {
        utils::publishVoronoiVertices(this->now(), vertices, voronoi_frame_,
                                      voronoi_vertices_pub_);
      }

      std::vector<double> dists(faces_vertices_.size(), 1.0);
      if (debug_acc == DEBUG_PRINT_VALUE) {
        // utils::computeMeanDistanceWithNearest(dists, container_pdf_, cloud_, true);
        utils::computeMeanDistancesWithNearest(dists, container_pdf_, cloud_, true);
        // utils::computeSphereDensity(dists, container_pdf_, cloud_, true);
      } else {
        // utils::computeMeanDistanceWithNearest(dists, container_pdf_, cloud_, false);
        utils::computeMeanDistancesWithNearest(dists, container_pdf_, cloud_, false);
        // utils::computeSphereDensity(dists, container_pdf_, cloud_, false);
      }

      for (int i = 0; i < mean_dist_with_nearest_.size(); i++) {

        // if (debug_) {
        //   std::cout << "Cell " << i << std::endl;
        //   std::cout << "\tCurrent distance" << ": " << mean_dist_with_nearest_[i] << std::endl;
        //   std::cout << "\t    New distance" << ": " << dists[i] / 0.01 << std::endl;
        // }

        mean_dist_with_nearest_[i] =
            std::min(mean_dist_with_nearest_[i], dists[i]);
      }

      Eigen::Quaterniond quat(
          r1_pose_.transform.rotation.w, r1_pose_.transform.rotation.x,
          r1_pose_.transform.rotation.y, r1_pose_.transform.rotation.z);
      auto rotmat = quat.toRotationMatrix();

      std::vector<double> angles;

      // compute the pdf coefficient
      for (size_t i = 0; i < faces_normals_.size(); i++) {
        angles.push_back(utils::angleBetweenNormals(
            faces_normals_[i], rotmat.col(2).normalized()));
      }
      auto topvalue_ids = utils::getTopKWithIndices(angles, 3);

      for (auto pair : topvalue_ids) {
        pdf_coeffs_[pair.second] =
            std::max(pair.first, pdf_coeffs_[pair.second]);
      }

      // if (debug_) {
      //   for (int i = 0; i < pdf_coeffs_.size(); i++) {q
      //     std::cout << "Face " << i << ": " << pdf_coeffs_[i] << std::endl;
      //   }
      // }
      
      // pdf_coeffs_[max_index] = max_value;
      
      std_msgs::msg::Float64MultiArray msg;
      
      // publish the pdf coefficients
      msg.data = pdf_coeffs_;
      pdf_coeffs_pub_->publish(msg);

      // publish the mean distance coefficients
      msg.data = mean_dist_with_nearest_;
      mean_min_dist_pub_->publish(msg);

      // publishTriangleList();

      auto base = tf_buffer_1_->lookupTransform(voronoi_frame_, base_frame_,
                                                tf2::TimePointZero, 10ms);

      std::vector<double> multipliers;
      for(size_t i=0; i<faces_centers_.size(); i++){
        // if (debug_) {
        //   std::cout << "i: " << i << std::endl;
        //   std::cout << "\tpdf_coeffs_[" << i << "]: " << pdf_coeffs_[i] << std::endl;
        //   std::cout << "\tmean_dist_with_nearest_[" << i << "]: " << mean_dist_with_nearest_[i] << std::endl;
        // }
        multipliers.push_back(ALPHA * (1-pdf_coeffs_[i]) + (1-ALPHA) * mean_dist_with_nearest_[i]);
      }



      // DEBUG
      // ============================================================================================================
      ++debug_acc;
      if (debug_acc>DEBUG_PRINT_VALUE) {
        if (debug_) {
          for (size_t i = 0; i < multipliers.size(); i++) {
            // std::cout << "multipliers[" << i << "]: " << multipliers[i] - prev_multipliers[i] << std::endl;
            std::cout << "multipliers[" << i << "]: " << multipliers[i] << std::endl;
          }
        std::cout << "--------------------------------" << std::endl;
        }

        // termination condition
        bool cond_1 = true;
        bool cond_2 = true;
        for (size_t i=0; i < multipliers.size(); i++) {
          if (multipliers[i] > 0.1) {
            cond_1 = false;
          }
          if (std::abs(multipliers[i] - prev_multipliers[i]) > 0.001) {
            cond_2 = false;
          }
        }
        if (cond_1 && cond_2) {
          std::cout << "\n\n\n" << "TERMINATION CONDITION MET\n\n" << std::endl; 
          exit(0);
        }

        prev_multipliers = multipliers;
        debug_acc = 0;
      }
      // ============================================================================================================



      auto res = utils::integrateVectorValuedPdfOverPolyhedron(
          vertices, container_, container_pdf_, j, multipliers);

      // if (debug_) {
      //   std::cout << "Integral Result: " << res.transpose() << std::endl;
      // }

      // project onto sphere surface
      res = utils::projectOnSphere(res, sphere_radius_);

      pose_.header.stamp = this->now();
      pose_.header.frame_id = voronoi_frame_;
      pose_.pose.position.x = res(0);  // r1_x;
      pose_.pose.position.y = res(1);  // r1_y;
      pose_.pose.position.z = res(2);  // r1_z;

      // publish the result
      simulateAnnealing();

      auto q = utils::computeQuaternion(
          Eigen::Vector3d(r1_x, r1_y, r1_z).normalized(),
          Eigen::Vector3d(0, 0, 0));
      if (prefix_1_.find('2') != std::string::npos) {
        // rotate q by 180 degrees on z axis
        Eigen::AngleAxisd angle_axis(M_PI, Eigen::Vector3d::UnitZ());
        q = q * Eigen::Quaterniond(angle_axis);
      }
      pose_.pose.orientation.x = q.x();
      pose_.pose.orientation.y = q.y();
      pose_.pose.orientation.z = q.z();
      pose_.pose.orientation.w = q.w();

      auto trans = tf_buffer_1_->lookupTransform(output_frame_, voronoi_frame_,
                                                 tf2::TimePointZero);
      tf2::doTransform(pose_, pose_, trans);

      target_pub_->publish(pose_);
      if (vertices.size() > 0) {
        vertices.clear();
      }

      // utils::publishMarker(this->now(), marker_array_, prefix_1_,
      //                      voronoi_frame_, faces_centers_, faces_normals_,
      //                      rotmat.col(2).normalized(), faces_vertices_,
      //                      pdf_coeffs_, markers_pub_);
      utils::publishMarker(this->now(), faces_vertices_, multipliers, prefix_1_,
                           voronoi_frame_, marker_array_, markers_pub_);
    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "CATCHING: Could not get transform: %s",
                   ex.what());
      // return;
    }
  }

  void simulateAnnealing() {
    // SIMULATED ANNEALIING:
    //    if the velocity is zero, we add a random noise to the pose
    Eigen::Vector3d r1_current_pose(r1_pose_.transform.translation.x,
                                    r1_pose_.transform.translation.y,
                                    r1_pose_.transform.translation.z);
    r1_trans_velocity_ = (r1_current_pose - r1_last_pose_) /
                         (this->now() - last_time_).seconds();
    // if(debug_){
    //   RCLCPP_INFO_STREAM(this->get_logger(), "Current pose: " <<
    //   r1_current_pose.transpose()); RCLCPP_INFO_STREAM(this->get_logger(),
    //   "Last pose: " << r1_last_pose_.transpose());
    // }
    if (r1_trans_velocity_.norm() < zero_vel_threshold_) {
      if (!annealing_) {
        if (debug_) {
          RCLCPP_INFO(this->get_logger(), "Starting annealing");
        }

        // generate a random perturbation over the sphere
        srand(time(NULL) + (int)(prefix_1_[0] - '0'));
        perturbation_ = Eigen::Vector3d::Random() * perturbation_scale_;
        perturbation_[2] = 0.0;

        annealing_start_time_ = this->now();
        annealing_ = true;
      }
    }

    // perform annealing until the flag is set to false
    if (annealing_) {
      geometry_msgs::msg::PoseStamped annealing_pose;
      annealing_pose.pose.position.x = perturbation_(0);
      annealing_pose.pose.position.y = perturbation_(1);
      annealing_pose.pose.position.z = perturbation_(2);

      auto trans = tf_buffer_1_->lookupTransform(voronoi_frame_, input_frame_,
                                                 tf2::TimePointZero, 10ms);
      tf2::doTransform(annealing_pose, pose_, trans);
      // project onto sphere surface
      Eigen::Vector3d res(pose_.pose.position.x, pose_.pose.position.y,
                          pose_.pose.position.z);
      res = utils::projectOnSphere(res, sphere_radius_);
      pose_.pose.position.x = res(0);
      pose_.pose.position.y = res(1);
      pose_.pose.position.z = res(2);
    }

    if ((this->now().nanoseconds() - annealing_start_time_.nanoseconds()) >
        annealing_duration_ * 1e9) {
      if (debug_ && annealing_) {
        RCLCPP_INFO(this->get_logger(), "Annealing finished");
      }
      annealing_ = false;
    }

    // Update the last pose and time
    r1_last_pose_ = r1_current_pose;
    last_time_ = this->now();
  }

  // subscribers and publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr
      voronoi_vertices_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      pdf_coeffs_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
      pdf_coeffs_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      point_cloud_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      markers_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      mean_min_dist_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
      mean_min_dist_sub_;

  rclcpp::TimerBase::SharedPtr timer_;
  // get robot end effector pose
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_1_, tf_buffer_2_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_1_, tf_listener_2_;
  geometry_msgs::msg::TransformStamped transform_;
  // utils
  geometry_msgs::msg::TransformStamped r1_pose_, r2_pose_;
  geometry_msgs::msg::PoseStamped pose_;
  std::vector<Eigen::Vector3d> vertices;

  visualization_msgs::msg::MarkerArray marker_array_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
  std::string input_frame_, input_frame_other_robot_;
  std::string prefix_1_, prefix_2_;
  std::string voronoi_frame_;
  std::string base_frame_;
  std::string output_frame_;
  std::string target_topic_;
  double center_x_, center_y_, center_z_;
  bool debug_;

  // simulated annealing
  geometry_msgs::msg::PoseStamped r1_velocity_;
  Eigen::Vector3d r1_last_pose_;
  Eigen::Vector3d r1_trans_velocity_;
  rclcpp::Time last_time_;
  rclcpp::Time annealing_start_time_;
  const double annealing_duration_ = 5.0; 
  Eigen::Vector3d perturbation_;
  Eigen::Quaterniond random_quaternion_;
  bool annealing_ = false;
  const double perturbation_scale_ = 0.05;
  const double zero_vel_threshold_ = 0.001;

  const double sphere_radius_ = 0.7;
  const double ALPHA = 0.0;
  // DEBUG
  int debug_acc = 0;
  std::vector<double> prev_multipliers;
  const int DEBUG_PRINT_VALUE = 100;

  // voronoi
  const double con_size_xmin = -2.0, con_size_xmax = 2.0;
  const double con_size_ymin = -2.0, con_size_ymax = 2.0;
  const double con_size_zmin = -2.0, con_size_zmax = 2.0;
  std::shared_ptr<voro::container> container_;
  std::shared_ptr<voro::container> container_pdf_;
  std::vector<std::shared_ptr<voro::wall_plane>> planes_, planes_pdf_;
  std::shared_ptr<voro::wall_sphere> sphere;
  std::vector<Eigen::Vector3d> faces_centers_;
  std::vector<Eigen::Vector3d> faces_normals_;
  std::map<int, std::vector<Eigen::Vector3d>> faces_vertices_;
  std::vector<double> pdf_coeffs_;
  std::vector<double> mean_dist_with_nearest_;
  // log
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VoronoiCalculator>());
  rclcpp::shutdown();
  return 0;
}
