#ifndef UTILS_H
#define UTILS_H
#define _USE_MATH_DEFINES

#include <math.h>

#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <numeric>
#include <tuple>
#include <vector>

#include "voro++.hh"

double test_pdf(double x, double y, double z) {
  Eigen::VectorXd result(3);
  result << x * y * z;
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

Eigen::Vector3d integrate_vector_valued_pdf_over_polyhedron(
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
            pdf = (1 - multipliers[tmp_cell_idx]) * test_pdf(x, y, z);
            // multivariate_gaussian_pdf(point, Eigen::Vector3d(0, 0, 0),
            //                           0.75 * Eigen::Matrix3d::Identity());
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

Eigen::Vector3d computeCenter(const std::vector<Eigen::Vector3d> &vertices,
                              const Eigen::Vector3d &center) {
  Eigen::Vector3d result(0, 0, 0);
  for (auto const &vertex : vertices) {
    result += vertex;
  }
  return result / vertices.size();
};

// passing array
void init_icosahedron_planes(
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

Eigen::Quaterniond computeQuaternion(const Eigen::Vector3d &point,
                                     const Eigen::Vector3d &center) {
  Eigen::Vector3d direction = (center - point).normalized();

  // Compute rotation axis by projecting direction onto XY plane
  Eigen::Vector3d projDirection =
      direction -
      direction.dot(Eigen::Vector3d::UnitZ()) * Eigen::Vector3d::UnitZ();
  double projDirectionNorm = projDirection.norm();
  Eigen::Vector3d axis =
      projDirectionNorm < 1e-5
          ? Eigen::Vector3d::UnitZ().cross(Eigen::Vector3d::UnitX())
          : projDirection.cross(Eigen::Vector3d::UnitZ());

  // Compute rotation angle
  double angle = -std::acos(direction.dot(Eigen::Vector3d::UnitZ()));

  // Create quaternion
  Eigen::Quaterniond q(Eigen::AngleAxisd(angle, axis.normalized()));
  return q.normalized();
}
Eigen::Vector3d projectOnSphere(Eigen::Vector3d pose, double radius) {
  auto pose_norm = pose.normalized();
  return radius * pose_norm;
}

// Function to generate a blues colormap with normalized color values
std::vector<double> generateBluesColormap(
    int numColors, const std::vector<double> &randomArray) {
  // Define the range of blue values
  double startBlue = 0.12;
  double endBlue = 1.0;

  // Calculate the increment between each color
  double increment = (endBlue - startBlue) / (numColors - 1);

  // Generate the colormap
  std::vector<double> coloredArray;
  for (double value : randomArray) {
    // Map the random value to the colormap index
    int index = value * numColors;
    // Calculate the current blue value
    double blueValue = startBlue + (index * increment);
    // Store the blue value in the colored array
    coloredArray.push_back(blueValue);
  }
  return coloredArray;
}

#endif