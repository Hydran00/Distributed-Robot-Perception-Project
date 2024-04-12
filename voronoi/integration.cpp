#include <iostream>
#include <vector>
#include <cmath>
#include <numeric>
#include <tuple>
#include <Eigen/Dense>
#include <boost/math/quadrature/gauss_kronrod.hpp>
#include <boost/math/constants/constants.hpp>

using namespace std;
using namespace Eigen;
namespace bmq = boost::math::quadrature;

// Define multivariate Gaussian PDF parameters
VectorXd mean(3); // Mean vector
MatrixXd covariance(3, 3); // Covariance matrix

// Multivariate Gaussian PDF in R^3
VectorXd multivariate_gaussian_pdf(double x, double y, double z) {
    Vector3d point(x, y, z);
    Vector3d diff = point - mean;
    double exponent = -0.5 * diff.transpose() * covariance.inverse() * diff;
    double normalization = pow(2 * boost::math::constants::pi<double>(), -mean.size() / 2) * sqrt(covariance.determinant());
    VectorXd density(3);
    density << normalization * exp(exponent), 0, 0; // Return a vector with the density for each component (in this case, just one component)
    return density;
}

// Define polyhedron volume
double polyhedron_volume(vector<Vector3d>& vertices) {
    MatrixXd mat(3, vertices.size());
    for (size_t i = 0; i < vertices.size(); ++i) {
        mat.col(i) = vertices[i];
    }
    return mat.jacobiSvd().singularValues().prod() / 6.0; // Using SVD to compute the volume of a tetrahedron
}

// Integrate multivariate Gaussian PDF over the polyhedron
VectorXd integrate_multivariate_gaussian_pdf_over_polyhedron(std::function<VectorXd(double, double, double)> pdf_func, vector<Vector3d>& vertices) {
    // Compute the volume of the polyhedron
    double polyhedron_vol = polyhedron_volume(vertices);

    // Define the function to integrate
    auto integrand = [pdf_func](double x, double y, double z) { return pdf_func(x, y, z); }; // Capture by value

    // Define the integration limits
    double xmin = numeric_limits<double>::infinity(), xmax = -numeric_limits<double>::infinity();
    double ymin = numeric_limits<double>::infinity(), ymax = -numeric_limits<double>::infinity();
    double zmin = numeric_limits<double>::infinity(), zmax = -numeric_limits<double>::infinity();
    for (const auto& vertex : vertices) {
        xmin = min(xmin, vertex(0));
        xmax = max(xmax, vertex(0));
        ymin = min(ymin, vertex(1));
        ymax = max(ymax, vertex(1));
        zmin = min(zmin, vertex(2));
        zmax = max(zmax, vertex(2));
    }

    // Perform the triple integral over the polyhedron using Gauss-Kronrod quadrature
    bmq::gauss_kronrod<double, 15> integrator;
    auto integrator_func = [&](double z) {
        auto integrator_func_y = [&](double y) {
            auto integrator_func_x = [&](double x) {
                return integrand(x, y, z);
            };
            return integrator.integrate(integrator_func_x, xmin, xmax);
        };
        return integrator.integrate(integrator_func_y, ymin, ymax);
    };
    VectorXd result(3);
    result << integrator.integrate(integrator_func, zmin, zmax), 0, 0; // Return a vector with the result for each component
    return result * polyhedron_vol; // Multiply each component by the volume of the polyhedron
}

int main() {
    // Set mean and covariance of the multivariate Gaussian PDF
    mean << 0, 0, 0; // Mean vector
    covariance = MatrixXd::Identity(3, 3); // Identity covariance matrix

    // Example usage: define the vertices of a tetrahedron
    vector<Vector3d> vertices = {Vector3d(0, 0, 0), Vector3d(1, 0, 0), Vector3d(0, 1, 0), Vector3d(0, 0, 1)};
    
    // Integrate the multivariate Gaussian PDF over the polyhedron
    VectorXd volume = integrate_multivariate_gaussian_pdf_over_polyhedron(multivariate_gaussian_pdf, vertices);
    cout << "Volume: " << volume.transpose() << endl;

    return 0;
}

