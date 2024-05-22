#ifndef UTILS_H
#define UTILS_H
#define _USE_MATH_DEFINES

#include <math.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#include <geometry_msgs/msg/pose_array.hpp>
#include <iostream>
#include <map>
#include <memory>
#include <numeric>
#include <rclcpp/rclcpp.hpp>
#include <tuple>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

#include "voro++.hh"

namespace utils {

// Custom mass function used to compute the center of mass of a polyhedron
double test_pdf(double x, double y, double z) {
  // Eigen::Vector3d current_pos(x, y, z);
  // if (z < 0) {
  //   return 0;
  // }

  // if ((robot_base - current_pos).norm() > radius) {
  //   return 0;
  // } else {
  //   return 1;
  //   // return 1 - (robot_base - current_pos).norm() / radius;
  // }
  return 1;
}

// Multivariate Gaussian PDF in R^3
double multivariate_gaussian_pdf(Eigen::Vector3d point, Eigen::Vector3d mean,
                                 Eigen::Matrix3d covariance) {
  Eigen::Vector3d diff = point - mean;
  double radius = 0.3;
  if (point.norm() > radius) {
    return 0;
  }
  double exponent = -0.5 * diff.transpose() * covariance.inverse() * diff;
  double normalization =
      pow(2 * M_PI, -mean.size() / 2) * sqrt(covariance.determinant());
  return normalization * exp(exponent);
}

// Function to compute integral of a vector valued pdf over a polyhedron
Eigen::Vector3d integrateVectorValuedPdfOverPolyhedron(
    std::vector<Eigen::Vector3d> &vertices,
    std::shared_ptr<voro::container> con,
    std::shared_ptr<voro::container> con_pdf, int cell_idx,
    std::vector<double> multipliers) {
  Eigen::Vector3d result(3);
  result << 0, 0, 0;

  double xmin = std::numeric_limits<double>::max(),
         xmax = std::numeric_limits<double>::min();
  double ymin = std::numeric_limits<double>::max(),
         ymax = std::numeric_limits<double>::min();
  double zmin = std::numeric_limits<double>::max(),
         zmax = std::numeric_limits<double>::min();

  // reduce the enclosing box for computing integral
  for (const auto &vertex : vertices) {
    xmin = std::min(xmin, vertex(0));
    xmax = std::max(xmax, vertex(0));
    ymin = std::min(ymin, vertex(1));
    ymax = std::max(ymax, vertex(1));
    zmin = std::min(zmin, vertex(2));
    zmax = std::max(zmax, vertex(2));
  }
  double x, y, z, rx, ry, rz;
  Eigen::Vector3d point;
  double h = 0.2;

  int tot = 0;
  double mass = 0, pdf = 0;
  // double cube_vol = pow(h, 3);
  Eigen::Vector3d com = Eigen::Vector3d::Zero();  // center of mass
  voro::c_loop_all cl(*con_pdf);
  voro::voronoicell c;
  int tmp_cell_idx;
  bool success;
  for (double ix = xmin + h / 2; ix < xmax; ix += h) {
    for (double iy = ymin + h / 2; iy < ymax; iy += h) {
      for (double iz = zmin + h / 2; iz < zmax; iz += h) {
        tot++;
        x = ix;
        y = iy;
        z = iz;
        point << x, y, z;
        // check if the point is inside the correct cell (one cell for each
        // robot)
        if (con->point_inside(x, y, z) &&
            con->find_voronoi_cell(x, y, z, rx, ry, rz, tmp_cell_idx)) {
          if (tmp_cell_idx != cell_idx) {
            continue;
          }
          // std::endl;
          if (con_pdf->find_voronoi_cell(x, y, z, rx, ry, rz, tmp_cell_idx)) {
            // pdf = multipliers[tmp_cell_idx] * test_pdf(x, y, z);
            // if ((1-multipliers[tmp_cell_idx]) < 0.6) {
            //   continue;
            // }
            pdf = multipliers[tmp_cell_idx] * test_pdf(x, y, z);
            // pdf = (1 - multipliers[tmp_cell_idx]) * test_pdf(x, y, z);
            // multivariate_gaussian_pdf(point, Eigen::Vector3d(0, 0, 0),
            //                           2.0 * Eigen::Matrix3d::Identity());
            mass += pdf;
            com += pdf * point;
          }
        }
      }
    }
  }
  // TODO: check that in equation 7 we can omit volume h^3
  return (1 / mass) * com;
}

// Function to compute the center of mass of a polyhedron
Eigen::Vector3d computeCenter(const std::vector<Eigen::Vector3d> &vertices,
                              const Eigen::Vector3d &center) {
  Eigen::Vector3d result(0, 0, 0);
  for (auto const &vertex : vertices) {
    result += vertex;
  }
  return result / vertices.size();
};

// Function used to cut a container into an icosahedron
void initIcosahedronPlanes(
    std::vector<std::shared_ptr<voro::wall_plane>> &walls, int scale = 1) {
  const double Phi = 0.5 * (1 + sqrt(5.0));
  const double phi = 0.5 * (1 - sqrt(5.0));
  std::shared_ptr<voro::wall_plane> p1 =
      std::make_shared<voro::wall_plane>(1, 1, 1, scale);
  walls.push_back(p1);
  std::shared_ptr<voro::wall_plane> p2 =
      std::make_shared<voro::wall_plane>(-1, 1, 1, scale);
  walls.push_back(p2);
  std::shared_ptr<voro::wall_plane> p3 =
      std::make_shared<voro::wall_plane>(1, -1, 1, scale);
  walls.push_back(p3);
  std::shared_ptr<voro::wall_plane> p4 =
      std::make_shared<voro::wall_plane>(-1, -1, 1, scale);
  walls.push_back(p4);
  std::shared_ptr<voro::wall_plane> p5 =
      std::make_shared<voro::wall_plane>(1, 1, -1, scale);
  walls.push_back(p5);
  std::shared_ptr<voro::wall_plane> p6 =
      std::make_shared<voro::wall_plane>(-1, 1, -1, scale);
  walls.push_back(p6);
  std::shared_ptr<voro::wall_plane> p7 =
      std::make_shared<voro::wall_plane>(1, -1, -1, scale);
  walls.push_back(p7);
  std::shared_ptr<voro::wall_plane> p8 =
      std::make_shared<voro::wall_plane>(-1, -1, -1, scale);
  walls.push_back(p8);
  std::shared_ptr<voro::wall_plane> p9 =
      std::make_shared<voro::wall_plane>(0, phi, Phi, scale);
  walls.push_back(p9);
  std::shared_ptr<voro::wall_plane> p10 =
      std::make_shared<voro::wall_plane>(0, phi, -Phi, scale);
  walls.push_back(p10);
  std::shared_ptr<voro::wall_plane> p11 =
      std::make_shared<voro::wall_plane>(0, -phi, Phi, scale);
  walls.push_back(p11);
  std::shared_ptr<voro::wall_plane> p12 =
      std::make_shared<voro::wall_plane>(0, -phi, -Phi, scale);
  walls.push_back(p12);
  std::shared_ptr<voro::wall_plane> p13 =
      std::make_shared<voro::wall_plane>(Phi, 0, phi, scale);
  walls.push_back(p13);
  std::shared_ptr<voro::wall_plane> p14 =
      std::make_shared<voro::wall_plane>(Phi, 0, -phi, scale);
  walls.push_back(p14);
  std::shared_ptr<voro::wall_plane> p15 =
      std::make_shared<voro::wall_plane>(-Phi, 0, phi, scale);
  walls.push_back(p15);
  std::shared_ptr<voro::wall_plane> p16 =
      std::make_shared<voro::wall_plane>(-Phi, 0, -phi, scale);
  walls.push_back(p16);
  std::shared_ptr<voro::wall_plane> p17 =
      std::make_shared<voro::wall_plane>(phi, Phi, 0, scale);
  walls.push_back(p17);
  std::shared_ptr<voro::wall_plane> p18 =
      std::make_shared<voro::wall_plane>(phi, -Phi, 0, scale);
  walls.push_back(p18);
  std::shared_ptr<voro::wall_plane> p19 =
      std::make_shared<voro::wall_plane>(-phi, Phi, 0, scale);
  walls.push_back(p19);
  std::shared_ptr<voro::wall_plane> p20 =
      std::make_shared<voro::wall_plane>(-phi, -Phi, 0, scale);
  walls.push_back(p20);
}

// Function to compute the angle between two normals
double angleBetweenNormals(const Eigen::Vector3d &normal1,
                           const Eigen::Vector3d &normal2) {
  auto normal1_ = normal1.normalized();
  auto normal2_ = normal2.normalized();

  // Compute the dot product of the two normals
  double dotProduct = normal1_.dot(normal2_);

  // Ensure dot product is within valid range [-1, 1]
  dotProduct = std::clamp(dotProduct, -1.0, 1.0);

  // Compute the angle in radians using arccosine
  return std::acos(dotProduct) / M_PI;

  // return euclidean distance
  // return 0.5 * sqrt(pow(normal1_(0) - normal2_(0), 2) +
  //             pow(normal1_(1) - normal2_(1), 2) +
  //             pow(normal1_(2) - normal2_(2), 2));
}

// Function to compute the quaternion that rotates the z-axis to the direction
Eigen::Quaterniond computeQuaternion(const Eigen::Vector3d &point,
                                     const Eigen::Vector3d &center) {
  const Eigen::Vector3d direction = (center - point).normalized();
  // Eigen::Vector3d axis = Eigen::Vector3d::UnitZ().cross(direction);
  // double angle = acos(Eigen::Vector3d::UnitZ().dot(direction));
  // Eigen::Quaterniond q;
  // q = Eigen::AngleAxisd(angle, axis.normalized());
  // return q;
  Eigen::Quaterniond q;
  Eigen::Vector3d up = Eigen::Vector3d::UnitZ();
  double dot = up.dot(direction);

  // Handle the case when the direction is parallel or anti-parallel to the up
  // vector
  if (std::abs(dot - (-1.0)) < 1.0e-10) {
    // 180 degrees rotation around any vector perpendicular to the target
    // direction
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ANTI-PARALLEL");
    Eigen::Vector3d axis = direction.unitOrthogonal();
    q = Eigen::Quaterniond::FromTwoVectors(up, axis);
  } else if (std::abs(dot - 1.0) < 1.0e-10) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PARALLEL");
    q = Eigen::Quaterniond::Identity();
  } else {
    // General case
    double angle = acos(dot);
    Eigen::Vector3d axis = up.cross(direction).normalized();
    q = Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis));
  }

  return q;
  // }
}

// Function to project a point on a sphere
Eigen::Vector3d projectOnSphere(Eigen::Vector3d pose, double radius) {
  auto pose_norm = pose.normalized();
  return radius * pose_norm;
}

// Function to generate a colormap with normalized color values
std::vector<double> generateColormap(int numColors,
                                     const std::vector<double> &randomArray) {
  // Define the range of values
  double start = 0.12;
  double end = 1.0;

  // Calculate the increment between each color
  double increment = (end - start) / (numColors - 1);

  // Generate the colormap
  std::vector<double> coloredArray;
  for (double value : randomArray) {
    // Map the random value to the colormap index
    int index = value * numColors;
    // Calculate the current  value
    double Value = start + (index * increment);
    // Store the  value in the colored array
    coloredArray.push_back(Value);
  }
  return coloredArray;
}

// Function to publish the voronoi vertices
void publishVoronoiVertices(
    const rclcpp::Time &stamp, const std::vector<Eigen::Vector3d> &vertices,
    const std::string &voronoi_frame_,
    const rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr
        voronoi_vertices_pub) {
  geometry_msgs::msg::PoseArray voronoi_vertices;

  // publish voronoi vertices
  voronoi_vertices.header.stamp = stamp;
  voronoi_vertices.header.frame_id = voronoi_frame_;
  voronoi_vertices.poses.clear();
  for (const auto &vertex : vertices) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = vertex(0);
    pose.position.y = vertex(1);
    pose.position.z = vertex(2);
    voronoi_vertices.poses.push_back(pose);
  }

  voronoi_vertices_pub->publish(voronoi_vertices);
}

// Function to publish a marker_array containing the voronoi vertices
void publishMarker(
    const rclcpp::Time &stamp,
    // const std::vector<Eigen::Vector3d> &face_centers,
    // const std::vector<Eigen::Vector3d> &face_normals,
    // const Eigen::Vector3d &current_rot_vec,
    std::map<int, std::vector<Eigen::Vector3d>> &faces_vertices,
    const std::vector<double> &pdf_coeffs, 
    const std::string &prefix_1,
    const std::string &frame_id,
    visualization_msgs::msg::MarkerArray &marker_array,
    const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        markers_publisher) {
  marker_array.markers.clear();
  visualization_msgs::msg::Marker mesh_msg;
  mesh_msg.header.frame_id = frame_id;
  mesh_msg.header.stamp = stamp;
  mesh_msg.ns = prefix_1;
  mesh_msg.id = 0;
  mesh_msg.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  mesh_msg.action = visualization_msgs::msg::Marker::MODIFY;
  mesh_msg.scale.x = 1.0;
  mesh_msg.scale.y = 1.0;
  mesh_msg.scale.z = 1.0;
  mesh_msg.color.r = 1.0;
  mesh_msg.color.a = 1.0;

  visualization_msgs::msg::Marker text_msg;
  text_msg.header.frame_id = frame_id;
  text_msg.header.stamp = stamp;
  text_msg.ns = prefix_1;
  text_msg.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text_msg.action = visualization_msgs::msg::Marker::MODIFY;
  text_msg.scale.z = 0.05;
  text_msg.color.r = 1.0;
  text_msg.color.g = 1.0;
  text_msg.color.b = 1.0;
  text_msg.color.a = 0.85;

  visualization_msgs::msg::Marker arrow_msg;
  arrow_msg.header.frame_id = frame_id;
  arrow_msg.header.stamp = stamp;
  arrow_msg.ns = prefix_1;
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
  std::vector<double> coloredArray;// =
      // generateColormap((int)pdf_coeffs.size(), pdf_coeffs);
  
  for (double value : pdf_coeffs) {
    coloredArray.push_back(1 - value);
  }

  // DEBUG
  // for (int i=0; i<coloredArray.size(); i++) {
  //   std::cout << "\t\t" << coloredArray[i] << std::endl;
  // }
  // std::cout << std::endl;

  geometry_msgs::msg::Point p;
  geometry_msgs::msg::Point normal;
  std_msgs::msg::ColorRGBA color;

  for (size_t i = 0; i < faces_vertices.size(); i++) {
    // auto vertices = faces_vertices[i];
    for (size_t j = 0; j < faces_vertices[i].size(); j++) {
      p.x = faces_vertices[i][j](0) / 6;
      p.y = faces_vertices[i][j](1) / 6;
      p.z = faces_vertices[i][j](2) / 6;
      mesh_msg.points.push_back(p);
    }
    color.r = coloredArray[i];
    color.g = coloredArray[i];
    color.b = coloredArray[i];
    color.a = 1.0;
    mesh_msg.colors.push_back(color);
    // compute face center
    Eigen::Vector3d face_center = Eigen::Vector3d::Zero();
    for (size_t j = 0; j < faces_vertices[i].size(); j++) {
      face_center += faces_vertices[i][j];
    }
    face_center /= faces_vertices[i].size();
    text_msg.pose.position.x = face_center(0) / 5;
    text_msg.pose.position.y = face_center(1) / 5;
    text_msg.pose.position.z = face_center(2) / 5;
    text_msg.text = std::to_string(i);
    text_msg.id = i + 1;
    texts.push_back(text_msg);
    arrow_msg.points.clear();
    // normal.x = 0.0;
    // normal.y = 0.0;
    // normal.z = 0.0;
    // arrow_msg.points.push_back(normal);
    // normal.x = face_normals[i](0);
    // normal.y = face_normals[i](1);
    // normal.z = face_normals[i](2);
    // arrow_msg.points.push_back(normal);
    // arrow_msg.id = (int)faces_vertices.size() + i + 1;
    // arrows.push_back(arrow_msg);
  }
  // // // CURRENT ORIENTATION
  // arrow_msg.points.clear();
  // normal.x = 0.0;
  // normal.y = 0.0;
  // normal.z = 0.0;
  // arrow_msg.points.push_back(normal);
  // normal.x = current_rot_vec(0);
  // normal.y = current_rot_vec(1);
  // normal.z = current_rot_vec(2);
  // arrow_msg.points.push_back(normal);
  // arrow_msg.id = 2 * (int)faces_vertices.size() + 1;
  // arrow_msg.color.g = 1.0;
  // arrow_msg.color.r = 0.0;

  // // std::cout << "/////////////////" << std::endl;
  marker_array.markers.push_back(mesh_msg);
  // for (auto text : texts) {
  //   marker_array.markers.push_back(text);
  // }
  // for (auto arrow : arrows) {
  //   marker_array.markers.push_back(arrow);
  // }
  // marker_array.markers.push_back(arrow_msg);
  markers_publisher->publish(marker_array);
}

// Function to get the top k coefficients of the bitmap
std::vector<std::pair<double, int>> getTopKWithIndices(
    const std::vector<double> &nums, int k) {
  std::vector<std::pair<double, int>> indexedNums(nums.size());
  for (int i = 0; i < nums.size(); ++i) {
    indexedNums[i] = {nums[i], i};
  }

  sort(indexedNums.rbegin(), indexedNums.rend());  // Sort in descending order

  std::vector<std::pair<double, int>> topkWithIndices;
  for (int i = 0; i < std::min(k, (int)indexedNums.size()); ++i) {
    topkWithIndices.push_back(indexedNums[i]);
  }
  return topkWithIndices;
}

void computeMeanDistanceWithNearest(std::vector<double> &mean_dist_with_nearest,
                                    std::shared_ptr<voro::container> container,
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool debug_print) {
  // variables initialization
  double x, y, z, rx, ry, rz;
  int cell_idx;
  std::vector<int> tot_points_per_cell;

  // set the current number of points per cell to 0
  for (int i = 0; i < container->total_particles(); i++) {
    tot_points_per_cell.push_back(0);
  }

  // first iteration to compute the mean distance with the nearest point
  for (int i = 0; i < cloud->size(); i++) {
    // populate x,y,z with point coordinates
    x = cloud->points[i].x;
    y = cloud->points[i].y;
    z = cloud->points[i].z;
    // find the voronoi cell that contains the point
    if (container->find_voronoi_cell(x, y, z, rx, ry, rz, cell_idx)) {
      // set current min distance to maximum value
      double min_dist = std::numeric_limits<double>::max();
      // iterate over all the points in the cloud to find the nearest point
      for (int j = 0; j < cloud->size(); j++) {
        // do not consider the same point
        if (i == j) {
          continue;
        }
        // compute distance between the two points
        double dist = sqrt(pow(x - cloud->points[j].x, 2) +
                           pow(y - cloud->points[j].y, 2) +
                           pow(z - cloud->points[j].z, 2));
        // update min distance if necessary
        if (dist < min_dist) {
          min_dist = dist;
        }
      }
      // update the sum of distances and the number of points per cell
      mean_dist_with_nearest[cell_idx] += min_dist;
      // update the number of points per cell
      tot_points_per_cell[cell_idx]++;
    }
  }
  
  // divide the sum of distances by the number of points per cell
  for (int i = 0; i < mean_dist_with_nearest.size(); i++) {
    if (tot_points_per_cell[i] == 0) {
      continue;
    }
    mean_dist_with_nearest[i] /= tot_points_per_cell[i];

    // set lower bound to distance
    if (mean_dist_with_nearest[i] < 0.01) {
      mean_dist_with_nearest[i] = 0.01;
    }
    // regularize distance
    mean_dist_with_nearest[i] -= 0.01;
    mean_dist_with_nearest[i] /= 10;

    // DEBUG
    if (debug_print) {
      std::cout << "Cell " << i << ": " << mean_dist_with_nearest[i]/0.01 << std::endl;
    }

  }

  // DEBUG
  if (debug_print) {
    std::cout << std::endl;
  }

}

// void computeMeanDistanceWithNearest(std::vector<double> &mean_dist_with_nearest,
//                                     std::shared_ptr<voro::container> container,
//                                     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
//   // variables initialization
//   double x, y, z, rx, ry, rz;
//   int cell_idx;
//   std::vector<int> tot_points_per_cell;
//   std::vector<int> cell_idxs;

//   // set the current number of points per cell to 0
//   for (int i = 0; i < container->total_particles(); i++) {
//     tot_points_per_cell.push_back(0);
//   }

//   // for each point of the point-cloud find the cell it belongs to
//   for (int i = 0; i < cloud->size(); i++) {
//     // populate x,y,z with point coordinates
//     x = cloud->points[i].x;
//     y = cloud->points[i].y;
//     z = cloud->points[i].z;
//     // find the voronoi cell that contains the point
//     if (container->find_voronoi_cell(x, y, z, rx, ry, rz, cell_idx)) {
//       cell_idxs.push_back(cell_idx);
//     } else {
//       cell_idxs.push_back(-1);
//     }
//   }

//   // first iteration to compute the mean distance with the nearest point
//   for (int i = 0; i < cloud->size(); i++) {
//     // populate x,y,z with point coordinates
//     x = cloud->points[i].x;
//     y = cloud->points[i].y;
//     z = cloud->points[i].z;
//     // find the voronoi cell that contains the point
//     if (cell_idxs[i] != -1) {
//       // set current min distance to maximum value
//       double min_dist = std::numeric_limits<double>::max();
//       // iterate over all the points in the cloud to find the nearest point
//       for (int j = 0; j < cloud->size(); j++) {
//         if (i != j && cell_idxs[j] == cell_idxs[i]) {
//           // compute distance between the two points
//           double dist = sqrt(pow(x - cloud->points[j].x, 2) +
//                             pow(y - cloud->points[j].y, 2) +
//                             pow(z - cloud->points[j].z, 2));
//           // update min distance if necessary
//           if (dist < min_dist) {
//             min_dist = dist;
//           }
//         }
//       }
//       // update the sum of distances and the number of points per cell
//       mean_dist_with_nearest[cell_idxs[i]] += min_dist;
//       // update the number of points per cell
//       tot_points_per_cell[cell_idxs[i]]++;
//     }
//   }
  
//   // divide the sum of distances by the number of points per cell
//   for (int i = 0; i < mean_dist_with_nearest.size(); i++) {
//     if (tot_points_per_cell[i] == 0) {
//       continue;
//     }
//     mean_dist_with_nearest[i] /= tot_points_per_cell[i];
//   }
// }

}  // namespace utils

#endif
