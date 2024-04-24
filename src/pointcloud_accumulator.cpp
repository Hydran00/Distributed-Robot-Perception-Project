// ROS2
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_srvs/srv/empty.hpp"
// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/transforms.hpp>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
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
            input_topic_1_, 1, std::bind(&PointCloudAccumulator::callback1, this, std::placeholders::_1));

        point_cloud_subscriber_2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_2_, 1, std::bind(&PointCloudAccumulator::callback2, this, std::placeholders::_1));

        // service for storing the pointcloud
        point_cloud_dump_service_ = this->create_service<std_srvs::srv::Empty>(
            "point_cloud_dump", std::bind(&PointCloudAccumulator::pointCloudDumpCallback, this, std::placeholders::_1, std::placeholders::_2));
        // Advertise the filtered point cloud topic
        total_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            output_topic_, 1);

        // Initialize the TF2 transform listener and buffer
        tf_buffer_1_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_1_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_1_);
        tf_buffer_2_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_2_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_2_);

        // initialize filters
        pass_z_local.setFilterFieldName("z");
        pass_z_local.setFilterLimits(0.0, 1.0);
        pass_z_target.setFilterFieldName("z");
        pass_z_target.setFilterLimits(0.02, 1.0);

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
    void callback1(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // id is used to distinguish the tf buffer
        processPointCloud(msg, 1);
    }
    void callback2(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // id is used to distinguish the tf buffer
        processPointCloud(msg, 2);
    }
    void pointCloudDumpCallback(
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response)
    {
        // suppress unused parameter warning
        (void)(request);
        (void)(response);
        // Dump the point cloud to a ply file
        pcl::io::savePLYFileBinary("total_cloud.ply", *total_cloud_);
        RCLCPP_INFO(this->get_logger(), "\033[96m \033[1m Point cloud has been dumped to total_cloud.ply \033[0m");
    }
    void processPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg, int id)
    {
        // transform_available = getTransform(total_cloud_frame_, msg->header.frame_id, msg->header.stamp, id);
        transform_available = getTransform(total_cloud_frame_, msg->header.frame_id, tf2::TimePointZero, id);

        if (!transform_available)
        {
            return;
        }

        // Convert the new sensor_msgs::PointCloud2 to pcl::PointCloud
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*msg, pcl_pc2);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

        // filter in the local frame
        localFrameFilter(cloud);

        // Apply the transform to the point cloud
        if (id == 1)
        {
            pcl::transformPointCloud(*cloud, *cloud, eigen_transform_1_);
        }
        else
        {
            pcl::transformPointCloud(*cloud, *cloud, eigen_transform_2_);
        }

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

    bool getTransform(
        const std::string &target_frame, const ::std::string &source_frame, const tf2::TimePoint &stamp, int id)
    {
        try
        {
            if (id == 1)
            {
                transform_1_ = tf_buffer_1_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
                pcl_ros::transformAsMatrix(transform_1_, eigen_transform_1_);
            }
            else
            {
                transform_2_ = tf_buffer_2_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
                pcl_ros::transformAsMatrix(transform_2_, eigen_transform_2_);
            }
            return true;
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to get transform: %s", ex.what());
            return false;
        }
    }

    void localFrameFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
    {
        // Apply PassThrough filter
        pass_z_local.setInputCloud(cloud);
        pass_z_local.filter(*cloud);
    }

    void filterPointCloud()
    {
        // Apply PassThrough filter
        pass_z_target.setInputCloud(total_cloud_);
        pass_z_target.filter(*total_cloud_);

        if ((int)total_cloud_->size() > max_cloud_size_)
        {
            // Apply VoxelGrid filter
            voxel_filter_.setInputCloud(total_cloud_);
            voxel_filter_.setLeafSize(voxel_grid_size, voxel_grid_size, voxel_grid_size);
            voxel_filter_.filter(*total_cloud_);

            // random sampling filter
            // random_sample.setInputCloud(total_cloud_);
            // random_sample.setSample(max_cloud_size_);
            // random_sample.filter(*total_cloud_);
        }
    }

    // subscribers and publishers
    std::string input_topic_1_, input_topic_2_, output_topic_, total_cloud_frame_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscriber_1_, point_cloud_subscriber_2_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr total_cloud_publisher_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr point_cloud_dump_service_;
    // tf conversion
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_1_, tf_buffer_2_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_1_, tf_listener_2_;
    geometry_msgs::msg::TransformStamped transform_1_, transform_2_;
    Eigen::Matrix4f eigen_transform_1_, eigen_transform_2_;
    // final cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr total_cloud_;
    sensor_msgs::msg::PointCloud2 total_cloud_msg_;

    // filters
    float voxel_grid_size;
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter_;
    int max_cloud_size_;
    pcl::RandomSample<pcl::PointXYZRGB> random_sample;
    pcl::PassThrough<pcl::PointXYZRGB> pass_z_target, pass_z_local;
    // utils
    bool transform_available;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudAccumulator>());
    rclcpp::shutdown();
    return 0;
}
