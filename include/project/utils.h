#ifndef UTILS_H
#define UTILS_H
#define _USE_MATH_DEFINES

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <numeric>
#include <tuple>
#include <vector>
#include <math.h>


Eigen::VectorXd test_pdf(double x, double y, double z) {
  Eigen::VectorXd result(3);
  result << x * y * z, x * y * z, x * y * z;
  return result;
}

struct Face {
  std::vector<Eigen::Vector3d> v;

  Eigen::Vector3d normal() const {
    assert(v.size() > 2);
    Eigen::Vector3d dir1 = v[1] - v[0];
    Eigen::Vector3d dir2 = v[2] - v[0];
    Eigen::Vector3d n = dir1.cross(dir2);
    double d = n.norm();
    return n / d;
  }
};

bool isInConvexPoly(const Eigen::Vector3d& p, const std::vector<Face>& fs) {
  for (const Face& f : fs) {
    Eigen::Vector3d p2f = f.v[0] - p;  // f.v[0] is an arbitrary point on f
    double d = p2f.dot(f.normal());
    d /= p2f.norm();  // for numeric stability

    constexpr double bound = -1e-15;  // use 1e15 to exclude boundaries
    if (d < bound) return false;
  }

  return true;
}

Eigen::Vector3d integrate_vector_valued_pdf_over_polyhedron(
    std::vector<Eigen::Vector3d>& vertices) {
  Eigen::Vector3d result(3);
  result << 0, 0, 0;

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
    ymin = std::min(ymin, vertex(0));
    ymax = std::max(ymax, vertex(0));
    zmin = std::min(zmin, vertex(0));
    zmax = std::max(zmax, vertex(0));
  }
  float x = 0.0;
  float y = 0.0;
  float z = 0.0;
  Eigen::Vector3d point;
  float h = 0.05;
  int counter = 0;
  int tot = 0;
  for (float ix = xmin + h / 2; ix < xmax; ix += h) {
    for (float iy = ymin + h / 2; iy < ymax; iy += h) {
      for (float iz = zmin + h / 2; iz < zmax; iz += h) {
        tot++;
        x = ix;
        y = iy;
        z = iz;
        point << x, y, z;
        // if (isInConvexPoly(point, vertices)) {
        result += test_pdf(x, y, z);
        // } else {
        //   // std::cout << "Point is not inside the polyhedron" << std::endl;
        //   // std::cout << "Point: " << point << std::endl;
        //   // std::cout << "---------------------" << std::endl;
        //   counter++;
        // }
      }
    }
  }
  //   std::cout << "Counter: " << counter << std::endl;
  //   std::cout << "Total: " << tot << std::endl;
  result *= pow(h, 3);
  return result;
}
// Multivariate Gaussian PDF in R^3
Eigen::VectorXd multivariate_gaussian_pdf(double x, double y, double z,
                                          Eigen::Vector3d mean,
                                          Eigen::Matrix3d covariance) {
  Eigen::Vector3d point(x, y, z);
  Eigen::Vector3d diff = point - mean;
  double exponent = -0.5 * diff.transpose() * covariance.inverse() * diff;
  double normalization =
      pow(2 * M_PI, -mean.size() / 2) *
      sqrt(covariance.determinant());
  Eigen::VectorXd density(3);
  density << normalization * exp(exponent), 0,
      0;  // Return a vector with the density for each component (in this case,
          // just one component)
  return density;
}

#endif