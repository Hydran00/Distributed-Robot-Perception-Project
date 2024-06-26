#include <Eigen/Dense>
#include <boost/math/constants/constants.hpp>
#include <boost/math/quadrature/gauss_kronrod.hpp>
#include <cmath>
#include <iostream>
#include <numeric>
#include <tuple>
#include <vector>
// ROS2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/convert.h"
#include "tf2/transform_datatypes.h"
// 1) integrate voro++ so that we can obtain vertices of the polyhedrons
// starting from robot pose 2) publish control input to the robot
using namespace std;
using namespace Eigen;
using namespace std::chrono_literals;
namespace bmq = boost::math::quadrature;

// Define multivariate Gaussian PDF parameters
VectorXd mean(3);           // Mean vector
MatrixXd covariance(3, 3);  // Covariance matrix

// Multivariate Gaussian PDF in R^3
VectorXd multivariate_gaussian_pdf(double x, double y, double z) {
  Vector3d point(x, y, z);
  Vector3d diff = point - mean;
  double exponent = -0.5 * diff.transpose() * covariance.inverse() * diff;
  double normalization =
      pow(2 * boost::math::constants::pi<double>(), -mean.size() / 2) *
      sqrt(covariance.determinant());
  VectorXd density(3);
  density << normalization * exp(exponent), 0,
      0;  // Return a vector with the density for each component (in this case,
          // just one component)
  return density;
}

// Product of [x, y, z] and the result of multivariate_gaussian_pdf(x, y, z)
VectorXd product_pdf(double x, double y, double z) {
  VectorXd pdf = multivariate_gaussian_pdf(x, y, z);
  VectorXd result = VectorXd::Zero(3);
  // Element-wise product with [x, y, z]
  result << x * pdf(0), y * pdf(0), z * pdf(0);  
  return result;
}

// Simple function to test f(x,y,z) = [-(x^2+y^2+z^2), -(x^2+y^2+z^2), -(x^2+y^2+z^2)]
VectorXd test_pdf(double x, double y, double z) {
  VectorXd result(3);
  result << -x * x - y * y - z * z, -x * x - y * y - z * z,
      -x * x - y * y - z * z;
  return result;
}

// Define polyhedron volume
double polyhedron_volume(vector<Vector3d> &vertices) {
  MatrixXd mat(3, vertices.size());
  for (size_t i = 0; i < vertices.size(); ++i) {
    mat.col(i) = vertices[i];
  }
  return mat.jacobiSvd().singularValues().prod() /
         6.0;  // Using SVD to compute the volume of a tetrahedron
}

// Find if a point is inside a polyhedron
bool point_inside_polyhedron(vector<Vector3d> &vertices, Vector3d &point) {
  // Define the polyhedron planes
  vector<Vector3d> planes;
  for (size_t i = 0; i < vertices.size(); ++i) {
    Vector3d v1 = vertices[i];
    Vector3d v2 = vertices[(i + 1) % vertices.size()];
    Vector3d normal = (v2 - v1).cross(v1 - point);
    normal.normalize();
    planes.push_back(normal);
  }

  // Check if the point is inside the polyhedron
  for (const auto &plane : planes) {
    double sum = 0;
    for (const auto &vertex : vertices) {
      sum += (vertex - point).dot(plane);
    }
    if (sum < 0) {
      return false;
    }
  }
  return true;
}

VectorXd integrate_vector_valued_pdf_over_polyhedron(
      std::function<VectorXd(double, double, double)> pdf_func,
      vector<Vector3d> &vertices, double polyhedron_vol) {
  
  VectorXd result(3);
  result << 0, 0, 0;

  double xmin = numeric_limits<double>::infinity(),
         xmax = -numeric_limits<double>::infinity();
  double ymin = numeric_limits<double>::infinity(),
         ymax = -numeric_limits<double>::infinity();
  double zmin = numeric_limits<double>::infinity(),
         zmax = -numeric_limits<double>::infinity();

  for (const auto &vertex : vertices) {
    xmin = min(xmin, vertex(0));
    xmax = max(xmax, vertex(0));
    ymin = min(ymin, vertex(0));
    ymax = max(ymax, vertex(0));
    zmin = min(zmin, vertex(0));
    zmax = max(zmax, vertex(0));
  }

  // Monte Carlo integration
  int num_samples = 1000;
  for (int i = 0; i < num_samples; ++i) {
    double x = xmin + (xmax - xmin) * rand() / RAND_MAX;
    double y = ymin + (ymax - ymin) * rand() / RAND_MAX;
    double z = zmin + (zmax - zmin) * rand() / RAND_MAX;
    Vector3d point(x, y, z);
    if (point_inside_polyhedron(vertices, point)) {
      result += product_pdf(x, y, z);
    }
  }

  result *= (polyhedron_vol / num_samples);


  return result;

}

class PdfIntegrator : public rclcpp::Node {
 public:
  PdfIntegrator() : Node("pdf_integrator") {
    // Publishers
    target_pub_1_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/robot1/target_frame", 10);
    target_pub_2_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/robot2/target_frame", 10);

    // Subscriber
    voronoi_sub_1_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/voronoi_vertices_1", 1,
        std::bind(&PdfIntegrator::callback_1, this, std::placeholders::_1));
    voronoi_sub_2_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/voronoi_vertices_2", 1,
        std::bind(&PdfIntegrator::callback_2, this, std::placeholders::_1));

    tf_buffer_1_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_1_ =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_1_);
    tf_buffer_2_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_2_ =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_2_);

    timer_ = this->create_wall_timer(
        20ms, std::bind(&PdfIntegrator::publish_target_frames, this));

    RCLCPP_INFO(this->get_logger(), "PdfIntegrator has been started.");
  }
  void publish_target_frames() {
    target_pub_1_->publish(pose_out_1_);
    target_pub_2_->publish(pose_out_2_);
  }
  void callback_1(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    cout << "--------------------------------" << endl;

    cout << "Callback 1" << endl;
    // extract volume which is the first element of the vector
    double volume = msg->poses[0].position.x;

    // Extract vertices from the message starting from the second element
    for (size_t i = 1; i < msg->poses.size(); ++i) {
      auto pose = msg->poses[i];
      vertices_1_.push_back(
          Vector3d(pose.position.x, pose.position.y, pose.position.z));
    }

    // Integrate the product of vector-valued PDF over the polyhedron
    result_1_ = integrate_vector_valued_pdf_over_polyhedron(
        product_pdf, vertices_1_, volume);
    cout << "1 -> Result: " << result_1_.transpose() << endl;

    vertices_1_.clear();
    // publish the result

    try {
      // transform vertices in the 1_base_link frame
      pose_in_1_.header.frame_id = "world";
      pose_in_1_.pose.position.x = result_1_(0);
      pose_in_1_.pose.position.y = result_1_(1);
      pose_in_1_.pose.position.z = result_1_(2);
      pose_in_1_.header.stamp = this->now();
      transform_1_ = tf_buffer_1_->lookupTransform("1_base_link", "world",
                                                   tf2::TimePointZero);
      tf2::doTransform(pose_in_1_, pose_out_1_, transform_1_);

      pose_out_1_.pose.position.x = -0.117;
      // pose_out_1_.pose.position.y = 0.328;
      pose_out_1_.pose.position.z = 0.253;

      pose_out_1_.pose.orientation.x = -0.707;
      pose_out_1_.pose.orientation.y = 0;
      pose_out_1_.pose.orientation.z = 0;
      pose_out_1_.pose.orientation.w = 0.707;
    } catch (tf2::TransformException &ex) {
      RCLCPP_INFO(this->get_logger(), "Could not get transform: %s", ex.what());
      // return;
    }
  }
  void callback_2(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    cout << "--------------------------------" << endl;

    cout << "Callback 2" << endl;

    // extract volume which is the first element of the vector
    double volume = msg->poses[0].position.x;

    // Extract vertices from the message starting from the second element
    for (size_t i = 1; i < msg->poses.size(); ++i) {
      auto pose = msg->poses[i];
      vertices_2_.push_back(
          Vector3d(pose.position.x, pose.position.y, pose.position.z));
    }

    // Integrate the product of vector-valued PDF over the polyhedron
    result_2_ = integrate_vector_valued_pdf_over_polyhedron(
        product_pdf, vertices_2_, volume);
    cout << "2 -> Result: " << result_2_.transpose() << endl;
    // publish the result

    vertices_2_.clear();

    try {
      // transform vertices in the 2_base_link frame
      pose_in_2_.header.frame_id = "world";
      pose_in_2_.pose.position.x = result_2_(0);
      pose_in_2_.pose.position.y = result_2_(1);
      pose_in_2_.pose.position.z = result_2_(2);
      pose_in_2_.header.stamp = this->now();
      transform_2_ = tf_buffer_2_->lookupTransform("2_base_link", "world",
                                                   tf2::TimePointZero);
      tf2::doTransform(pose_in_2_, pose_out_2_, transform_2_);

      pose_out_2_.pose.position.x = -0.117;
      // pose_out_2_.pose.position.y = 0.328;
      pose_out_2_.pose.position.z = 0.253;

      pose_out_2_.pose.orientation.x = -0.707;
      pose_out_2_.pose.orientation.y = 0;
      pose_out_2_.pose.orientation.z = 0;
      pose_out_2_.pose.orientation.w = 0.707;
    } catch (tf2::TransformException &ex) {
      RCLCPP_INFO(this->get_logger(), "Could not get transform: %s", ex.what());
      // return;
    }
  }

 private:
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_1_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_2_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr voronoi_sub_1_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr voronoi_sub_2_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::PoseStamped target_1_, target_2_;
  VectorXd result_1_;
  VectorXd result_2_;
  vector<Vector3d> vertices_1_, vertices_2_;
  // transform frame
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_1_, tf_buffer_2_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_1_, tf_listener_2_;
  geometry_msgs::msg::TransformStamped transform_1_, transform_2_;
  geometry_msgs::msg::PoseStamped pose_in_1_, pose_out_1_;
  geometry_msgs::msg::PoseStamped pose_in_2_, pose_out_2_;
};

int main(int argc, char *argv[]) {
  // Set mean and covariance of the multivariate Gaussian PDF
  mean << 0, 0, 0;                        // Mean vector
  covariance = MatrixXd::Identity(3, 3);  // Identity covariance matrix

  // // Example usage: define the vertices of a tetrahedron
  // vector<Vector3d> vertices = {Vector3d(0, 0, 0), Vector3d(1, 0, 0),
  // Vector3d(0, 1, 0), Vector3d(0, 0, 1)};

  // // Integrate the product of vector-valued PDF over the polyhedron
  // VectorXd result = integrate_vector_valued_pdf_over_polyhedron(product_pdf,
  // vertices); cout << "Result: " << result.transpose() << endl;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PdfIntegrator>());
  rclcpp::shutdown();
  return 0;
}
