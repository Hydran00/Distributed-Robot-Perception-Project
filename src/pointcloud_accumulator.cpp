#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <pcl_ros/transforms.hpp>
// eigen
#include <Eigen/Core>
using namespace std::chrono_literals;
class PointCloudAccumulator : public rclcpp::Node
{
public:
    PointCloudAccumulator() : Node("point_cloud_filter_node")
    {
        // avoid storing the pointcloud if transform is not available
        rclcpp::sleep_for(2s);

        // Initialize the filtered point cloud
        total_cloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

        // Set maximum size for the filtered point cloud

        this->declare_parameter("voxel_grid_size", 0.01);
        voxel_grid_size = (float)this->get_parameter("voxel_grid_size").as_double();

        this->declare_parameter("max_cloud_size", 10000);
        max_cloud_size_ = this->get_parameter("max_cloud_size").as_int();

        this->declare_parameter("input_topic_1", "/cloud_out_1");
        input_topic_1_ = this->get_parameter("input_topic_1").as_string();

        this->declare_parameter("input_topic_2", "/cloud_out_2");
        input_topic_2_ = this->get_parameter("input_topic_2").as_string();

        this->declare_parameter("output_topic", "/total_cloud");
        output_topic_ = this->get_parameter("output_topic").as_string();

        this->declare_parameter("frame_id", "world");
        total_cloud_frame_ = this->get_parameter("frame_id").as_string();

        // Subscribe to the point cloud topic
        point_cloud_subscriber_1_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_1_, 1, std::bind(&PointCloudAccumulator::pointCloudCallback, this, std::placeholders::_1));

        point_cloud_subscriber_2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_2_, 1, std::bind(&PointCloudAccumulator::pointCloudCallback, this, std::placeholders::_1));

        // Advertise the filtered point cloud topic
        total_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            output_topic_, 1);

        // Initialize the TF2 transform listener and buffer
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // log every param
        RCLCPP_INFO(this->get_logger(), "input_topic_1: %s", input_topic_1_.c_str());
        RCLCPP_INFO(this->get_logger(), "input_topic_2: %s", input_topic_2_.c_str());
        RCLCPP_INFO(this->get_logger(), "output_topic: %s", output_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "total_cloud_frame: %s", total_cloud_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "voxel_grid_size: %f", voxel_grid_size);
        RCLCPP_INFO(this->get_logger(), "max_cloud_size: %d", max_cloud_size_);

        RCLCPP_INFO(this->get_logger(), "PointCloudAccumulator has been started.");
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {

        bool res = transformPointCloud(total_cloud_frame_, *msg, *msg, *tf_buffer_);
        if (!res)
        {
            return;
        }

        // Convert sensor_msgs::PointCloud2 to pcl::PointCloud
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*msg, pcl_pc2);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

        // Add the received point cloud to the filtered point cloud
        *total_cloud_ += *cloud;

        // Filter the filtered point cloud based on size
        filterPointCloud();

        // Publish the filtered point cloud
        pcl::toROSMsg(*total_cloud_, total_cloud_msg_);
        total_cloud_msg_.header = msg->header;
        total_cloud_msg_.header.frame_id = total_cloud_frame_;
        total_cloud_publisher_->publish(total_cloud_msg_);
    }

    bool transformPointCloud(
        const std::string &target_frame, const sensor_msgs::msg::PointCloud2 &in,
        sensor_msgs::msg::PointCloud2 &out, const tf2_ros::Buffer &tf_buffer)
    {
        if (in.header.frame_id == target_frame)
        {
            out = in;
            return true;
        }
        try
        {
            transform_ = tf_buffer.lookupTransform("world", "1_camera", tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to get transform: %s", ex.what());
            return false;
        }

        // Convert the TF transform to Eigen format
        pcl_ros::transformAsMatrix(transform_, eigen_transform);
        // print transform
        // std::cout << "Transform: " << std::endl;
        // std::cout << eigen_transform << std::endl;

        pcl_ros::transformPointCloud(eigen_transform, in, out);
        out.header.frame_id = target_frame;
        return true;
    }

    void filterPointCloud()
    {
        RCLCPP_INFO(this->get_logger(), "Size is %d", (int)total_cloud_->size());
        if ((int)total_cloud_->size() > max_cloud_size_)
        {
            // Apply VoxelGrid filter
            // voxel_filter_.setInputCloud(total_cloud_);
            // voxel_filter_.setLeafSize(voxel_grid_size, voxel_grid_size, voxel_grid_size);
            // voxel_filter_.filter(*total_cloud_);

            // random sampling filter
            random_sample.setInputCloud(total_cloud_);
            random_sample.setSample(max_cloud_size_);
        }
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr total_cloud_;
    int max_cloud_size_;
    float voxel_grid_size;
    std::string input_topic_1_, input_topic_2_, output_topic_, total_cloud_frame_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscriber_1_, point_cloud_subscriber_2_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr total_cloud_publisher_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    sensor_msgs::msg::PointCloud2 total_cloud_msg_;

    // filters
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter_;
    pcl::RandomSample<pcl::PointXYZRGB> random_sample;

    // Get the TF transform
    geometry_msgs::msg::TransformStamped transform_;
    Eigen::Matrix4f eigen_transform;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudAccumulator>());
    rclcpp::shutdown();
    return 0;
}
