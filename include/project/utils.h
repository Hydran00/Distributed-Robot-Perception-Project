#ifndef UTILS_H
#define UTILS_H
#define _USE_MATH_DEFINES

#include <math.h>

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <memory>
#include <numeric>
#include <tuple>
#include <vector>

#include "voro++.hh"

Eigen::VectorXd test_pdf(double x, double y, double z) {
  Eigen::VectorXd result(3);
  result << x * y * z, x * y * z, x * y * z;
  return result;
}

// Multivariate Gaussian PDF in R^3
Eigen::VectorXd multivariate_gaussian_pdf(Eigen::Vector3d point,
                                          Eigen::Vector3d mean,
                                          Eigen::Matrix3d covariance) {
  Eigen::Vector3d diff = point - mean;
  double exponent = -0.5 * diff.transpose() * covariance.inverse() * diff;
  double normalization =
      pow(2 * M_PI, -mean.size() / 2) * sqrt(covariance.determinant());
  Eigen::VectorXd density(3);
  density << normalization * exp(exponent), 0,
      0;  // Return a vector with the density for each component (in this case,
          // just one component)
  return density;
}

Eigen::Vector3d integrate_vector_valued_pdf_over_polyhedron(
    std::vector<Eigen::Vector3d>& vertices,
    std::shared_ptr<voro::container> con, int cell_idx) {
  Eigen::Vector3d result(3);
  result << 0, 0, 0;

  // std::cout << "Vertices: " << vertices.size() << std::endl;
  // std::cout << "Cell idx: " << cell_idx << std::endl;

  double xmin = std::numeric_limits<double>::max(),
         xmax = std::numeric_limits<double>::min();
  double ymin = std::numeric_limits<double>::max(),
         ymax = std::numeric_limits<double>::min();
  double zmin = std::numeric_limits<double>::max(),
         zmax = std::numeric_limits<double>::min();

  // reduce the enclosing box for computing integral
  for (const auto& vertex : vertices) {
    xmin = std::min(xmin, vertex(0));
    xmax = std::max(xmax, vertex(0));
    ymin = std::min(ymin, vertex(1));
    ymax = std::max(ymax, vertex(1));
    zmin = std::min(zmin, vertex(2));
    zmax = std::max(zmax, vertex(2));
  }
  double x, y, z, rx, ry, rz;
  Eigen::Vector3d point;
  double h = 0.05;
  int counter = 0;
  int tot = 0;
  // std::cout <<"xmin: "<<xmin<<std::endl;
  // std::cout <<"xmax: "<<xmax<<std::endl;
  // std::cout <<"ymin: "<<ymin<<std::endl;
  // std::cout <<"ymax: "<<ymax<<std::endl;
  // std::cout <<"zmin: "<<zmin<<std::endl;
  // std::cout <<"zmax: "<<zmax<<std::endl;

  for (double ix = xmin + h / 2; ix < xmax; ix += h) {
    for (double iy = ymin + h / 2; iy < ymax; iy += h) {
      for (double iz = zmin + h / 2; iz < zmax; iz += h) {
        tot++;
        x = ix;
        y = iy;
        z = iz;
        point << x, y, z;
        // std::cout << "\t\tstart"<<std::endl;
        if (con->find_voronoi_cell(x, y, z, rx, ry, rz, cell_idx)) {
          result += multivariate_gaussian_pdf(point, Eigen::Vector3d(0, 0, 0.5),
                                              Eigen::Matrix3d::Identity());
        }
        // std::cout << "\t\tend"<<std::endl;
      }
    }
  }
  // std::cout << "Counter: " << counter << std::endl;
  // std::cout << "Total: " << tot << std::endl;
  result *= pow(h, 3);
  // std::cout << " return" << std::endl;
  return result;
}

#endif