#include <iostream>
#include <vector>
#include <cmath>
#include <numeric>
#include <tuple>
#include <Eigen/Dense>
#include <boost/math/quadrature/gauss_kronrod.hpp>
#include <boost/math/constants/constants.hpp>
// ROS2
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
// 1) integrate voro++ so that we can obtain vertices of the polyhedrons starting from robot pose
// 2) publish control input to the robot
using namespace std;
using namespace Eigen;
using namespace std::chrono_literals;
namespace bmq = boost::math::quadrature;

// Define multivariate Gaussian PDF parameters
VectorXd mean(3);          // Mean vector
MatrixXd covariance(3, 3); // Covariance matrix

// Multivariate Gaussian PDF in R^3
VectorXd multivariate_gaussian_pdf(double x, double y, double z)
{
    Vector3d point(x, y, z);
    Vector3d diff = point - mean;
    double exponent = -0.5 * diff.transpose() * covariance.inverse() * diff;
    double normalization = pow(2 * boost::math::constants::pi<double>(), -mean.size() / 2) * sqrt(covariance.determinant());
    VectorXd density(3);
    density << normalization * exp(exponent), 0, 0; // Return a vector with the density for each component (in this case, just one component)
    return density;
}

// Product of [x, y, z] and the result of multivariate_gaussian_pdf(x, y, z)
VectorXd product_pdf(double x, double y, double z)
{
    VectorXd pdf = multivariate_gaussian_pdf(x, y, z);
    VectorXd result = VectorXd::Zero(3);
    result << x * pdf(0), y * pdf(0), z * pdf(0); // Element-wise product with [x, y, z]
    return result;
}

// Define polyhedron volume
double polyhedron_volume(vector<Vector3d> &vertices)
{
    MatrixXd mat(3, vertices.size());
    for (size_t i = 0; i < vertices.size(); ++i)
    {
        mat.col(i) = vertices[i];
    }
    return mat.jacobiSvd().singularValues().prod() / 6.0; // Using SVD to compute the volume of a tetrahedron
}

// Integrate vector-valued PDF over the polyhedron
VectorXd integrate_vector_valued_pdf_over_polyhedron(std::function<VectorXd(double, double, double)> pdf_func, vector<Vector3d> &vertices)
{
    // Compute the volume of the polyhedron
    double polyhedron_vol = polyhedron_volume(vertices);

    // Define the integration limits
    double xmin = numeric_limits<double>::infinity(), xmax = -numeric_limits<double>::infinity();
    double ymin = numeric_limits<double>::infinity(), ymax = -numeric_limits<double>::infinity();
    double zmin = numeric_limits<double>::infinity(), zmax = -numeric_limits<double>::infinity();
    for (const auto &vertex : vertices)
    {
        xmin = min(xmin, vertex(0));
        xmax = max(xmax, vertex(0));
        ymin = min(ymin, vertex(1));
        ymax = max(ymax, vertex(1));
        zmin = min(zmin, vertex(2));
        zmax = max(zmax, vertex(2));
    }

    // Perform the triple integral over the polyhedron for each component separately
    VectorXd result(3);
    for (int i = 0; i < 3; ++i)
    {
        auto integrator_func = [&](double z)
        {
            auto integrator_func_y = [&](double y)
            {
                auto integrator_func_x = [&](double x)
                {
                    return pdf_func(x, y, z)(i); // Extract the ith component
                };
                return bmq::gauss_kronrod<double, 15>().integrate(integrator_func_x, xmin, xmax);
            };
            return bmq::gauss_kronrod<double, 15>().integrate(integrator_func_y, ymin, ymax);
        };
        result(i) = bmq::gauss_kronrod<double, 15>().integrate(integrator_func, zmin, zmax);
    }

    // Multiply each component by the volume of the polyhedron
    return result * polyhedron_vol;
}

class PdfIntegrator : public rclcpp::Node
{
public:
    PdfIntegrator() : Node("pdf_integrator")
    {
        // Publishers
        target_pub_1_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/robot1/target_frame", 10);
        target_pub_2_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/robot2/target_frame", 10);

        // Subscriber
        voronoi_sub_1_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/voronoi_vertices_1", 10, std::bind(&PdfIntegrator::callback_1, this, std::placeholders::_1));
        voronoi_sub_2_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/voronoi_vertices_2", 10, std::bind(&PdfIntegrator::callback_2, this, std::placeholders::_1));

        tf_buffer_1_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_1_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_1_);
        tf_buffer_2_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_2_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_2_);

        RCLCPP_INFO(this->get_logger(), "PdfIntegrator has been started.");
    }
    void callback_1(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        // Extract vertices from the message
        for (const auto &pose : msg->poses)
        {
            vertices_1_.push_back(Vector3d(pose.position.x, pose.position.y, pose.position.z));
        }

        // Integrate the product of vector-valued PDF over the polyhedron
        result_1_ = integrate_vector_valued_pdf_over_polyhedron(product_pdf, vertices_1_);
        cout << "1 -> Result: " << result_1_.transpose() << endl;
        // publish the result

        try
        {
            transform_1_ = tf_buffer_1_->lookupTransform("1_base_link", "world", tf2::TimePointZero);
            // apply the transform to the vertices
            for (auto &vertex : vertices_1_)
            {
                transformPoint(vertex, transform_1_);
            }
        }
        catch (tf2::TransformException &ex)
        {
            return;
        }
        geometry_msgs::msg::PoseStamped target_1_;
        target_1_.pose.position.x = result_1_(0);
        target_1_.pose.position.y = result_1_(1);
        target_1_.pose.position.z = result_1_(2);
        target_pub_1_->publish(target_1_);
        vertices_1_.clear();
    }
    void callback_2(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        // Extract vertices from the message
        for (const auto &pose : msg->poses)
        {
            vertices_2_.push_back(Vector3d(pose.position.x, pose.position.y, pose.position.z));
        }
        try
        {
            transform_2_ = tf_buffer_2_->lookupTransform("2_base_link", "world", tf2::TimePointZero);
            // apply the transform to the vertices
            for (auto &vertex : vertices_2_)
            {
                transformPoint(vertex, transform_2_);
            }
        }
        catch (tf2::TransformException &ex)
        {
            return;
        }

        // Integrate the product of vector-valued PDF over the polyhedron
        result_2_ = integrate_vector_valued_pdf_over_polyhedron(product_pdf, vertices_2_);
        cout << "2 -> Result: " << result_2_.transpose() << endl;
        // publish the result
        geometry_msgs::msg::PoseStamped target_2_;
        target_2_.pose.position.x = result_2_(0);
        target_2_.pose.position.y = result_2_(1);
        target_2_.pose.position.z = result_2_(2);
        target_pub_2_->publish(target_2_);
        vertices_2_.clear();
    }
    void transformPoint(Vector3d &point, geometry_msgs::msg::TransformStamped &transform)
    {
        Vector3d transformed_point = Vector3d(transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z) * point +
                                     Vector3d(transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z);
        point << transformed_point(0), transformed_point(1), transformed_point(2);
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_1_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_2_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr voronoi_sub_1_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr voronoi_sub_2_;
    geometry_msgs::msg::PoseStamped target_1_, target_2_;
    VectorXd result_1_;
    VectorXd result_2_;
    vector<Vector3d> vertices_1_, vertices_2_;
    // transform frame
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_1_, tf_buffer_2_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_1_, tf_listener_2_;
    geometry_msgs::msg::TransformStamped transform_1_, transform_2_;
};

int main(int argc, char *argv[])
{
    // Set mean and covariance of the multivariate Gaussian PDF
    mean << 0, 0, 0;                       // Mean vector
    covariance = MatrixXd::Identity(3, 3); // Identity covariance matrix

    // // Example usage: define the vertices of a tetrahedron
    // vector<Vector3d> vertices = {Vector3d(0, 0, 0), Vector3d(1, 0, 0), Vector3d(0, 1, 0), Vector3d(0, 0, 1)};

    // // Integrate the product of vector-valued PDF over the polyhedron
    // VectorXd result = integrate_vector_valued_pdf_over_polyhedron(product_pdf, vertices);
    // cout << "Result: " << result.transpose() << endl;

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PdfIntegrator>());
    rclcpp::shutdown();
    return 0;
}
