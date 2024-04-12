#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class PointCloudFilterNode : public rclcpp::Node
{
public:
    PointCloudFilterNode() : Node("point_cloud_filter_node")
    {
        // Subscribe to the point cloud topic
        point_cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cloud_out_1", 1, std::bind(&PointCloudFilterNode::pointCloudCallback, this, std::placeholders::_1));

        // Initialize the filtered point cloud
        total_cloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

        // Set maximum size for the filtered point cloud
        max_cloud_size_ = 10000; // Maximum number of points allowed

        this->declare_parameter("voxel_grid_size", 0.01);
        voxel_grid_size = (float)this->get_parameter("voxel_grid_size").as_double();

        // Advertise the filtered point cloud topic
        total_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/total_cloud", 1);
        
        
        
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
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
        total_cloud_publisher_->publish(total_cloud_msg_);
    }

    void filterPointCloud()
    {
        if (total_cloud_->size() > max_cloud_size_)
        {
            // Apply VoxelGrid filter
            voxel_filter_.setInputCloud(total_cloud_);
            voxel_filter_.setLeafSize(voxel_grid_size,voxel_grid_size,voxel_grid_size);
            voxel_filter_.filter(*total_cloud_);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr total_cloud_publisher_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr total_cloud_;
    sensor_msgs::msg::PointCloud2 total_cloud_msg_;
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter_;
    size_t max_cloud_size_;
    float voxel_grid_size;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudFilterNode>());
    rclcpp::shutdown();
    return 0;
}
