
#include <iostream>

#include "project/utils.h"
// ROS2
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
// PCL ROS
#include <pcl/point_cloud.h>
#include <pcl/surface/convex_hull.h>
#include <pcl_conversions/pcl_conversions.h>

#include "visualization_msgs/msg/marker_array.hpp"
// tf2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "tf2/convert.h"

using namespace std;
using namespace std::chrono_literals;

class VoronoiCalculator : public rclcpp::Node {
 public:
  VoronoiCalculator() : Node("voronoi_calculator") {
    // avoid storing the pointcloud if transform is not available

    point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/total_cloud", 1,
        std::bind(&VoronoiCalculator::pointCloudCallback, this,
                  std::placeholders::_1));
    markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/markers1_", 1);
    timer_ = this->create_wall_timer(
        20ms, std::bind(&VoronoiCalculator::updateVoronoi, this));
    cloud_ =
        pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  }

 private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    pcl::fromROSMsg(*msg, *cloud_);
    // unsubscribe
  }
  void updateVoronoi() {
    if (cloud_->size() == 0) {
      return;
    }
    // compute convex hull
    pcl::ConvexHull<pcl::PointXYZ> chull;
    std::vector<pcl::Vertices> polygons;
    // copy the current cloud to avoid concurrency issues
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cpy =
        pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr hull_points =
        pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::copyPointCloud(*cloud_, *cloud_cpy);
    chull.setInputCloud(cloud_cpy);
    chull.reconstruct(*hull_points, polygons);

    // clear map
    faces_vertices_.clear();

    std::cout << "Polygons size: " << polygons.size() << std::endl;
    if (polygons.size() == 0) {
      return;
    }
    // extract face vertices
    for (std::size_t i = 0; i < polygons.size(); ++i) {
      const pcl::Vertices &face1 = polygons[i];
      std::vector<Eigen::Vector3d> tmp;
      // iterate over the vertices of a face
      for (std::size_t j = 0; j < face1.vertices.size(); ++j) {
        const pcl::PointXYZ &pt = hull_points->points[face1.vertices[j]];
        tmp.push_back(Eigen::Vector3d(pt.x, pt.y, pt.z));
      }
      faces_vertices_[i] = tmp;
    }

    std::vector<double> pdf_coeffs;
    if (is_first_cloud_) {
      int i = 0;
      for (auto &face : faces_vertices_) {
        pdf_coeffs.push_back(i / (double)faces_vertices_.size());
        i++;
      }
      cloud_first_ = cloud_;
      faces_vertices_first_ = faces_vertices_;
      pdf_coeffs_first_ = pdf_coeffs;
      is_first_cloud_ = false;
      std::cout << "First cloud stored" << std::endl;
      return;
    }
    int min_face_idx = -1;

    pdf_coeffs.clear();
    std::cout << "Computing pdf coeffs" << std::endl;
    for (auto &face : faces_vertices_) {
      // get the nearest face with respect to first cloud
      double min_dist = std::numeric_limits<double>::max();
      int min_face = -1;

      Eigen::Vector3d center_curr(0, 0, 0);
      for (auto &v : face.second) {
        center_curr += v;
      }
      center_curr /= 3.0;
      for (auto &face_first : faces_vertices_first_) {
        double dist = 0;
        // compute face center of first cloud
        Eigen::Vector3d center_first(0, 0, 0);
        for (auto &v : face_first.second) {
          center_first += v;
        }
        center_first /= 3.0;
        dist = (center_first - center_curr).norm();
        if (dist < min_dist) {
          min_dist = dist;
          min_face = face_first.first;
          //   min_face = face_first.first;
          // compare
        }
      }
      pdf_coeffs.push_back(pdf_coeffs_first_[min_face]);
    }
    std::cout << "Coeff size " << pdf_coeffs.size() <<" vs "<< faces_vertices_.size()<< std::endl;

    utils::publishMarker(this->now(), faces_vertices_, pdf_coeffs, "1_",
                         "world", marker_array_, markers_pub_);
    std::cout << "------" << std::endl;
  }

 private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      point_cloud_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      markers_pub_;
  visualization_msgs::msg::MarkerArray marker_array_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_, cloud_first_;

  // voronoi
  std::map<int, std::vector<Eigen::Vector3d>> faces_vertices_,
      faces_vertices_first_;
  bool is_first_cloud_ = true;
  std::vector<double> pdf_coeffs_first_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VoronoiCalculator>());
  rclcpp::shutdown();
  return 0;
}