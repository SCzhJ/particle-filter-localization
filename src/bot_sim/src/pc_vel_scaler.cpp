#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// Parameters
double heading_angle_rad = 0; 
double angle_threshold_rad = M_PI / 36;
double voxel_leaf_size = 0.05;  // Leaf size for downsampling
double dist = 2.2;

double slope_likelihood(PointCloud::Ptr cloud, double slope_angle) {
    ROS_INFO("size: %ld", cloud->points.size());
    double sum = 0;
    for (const auto& point : cloud->points) {
        double dx = point.x;
        double dz = point.z;
        sum += dx * std::cos(slope_angle) + dz * std::sin(slope_angle);
    }
    double avg = sum / cloud->points.size();
    double var = 0;
    for (const auto& point : cloud->points) {
        double dx = point.x;
        double dz = point.z;
        var += std::pow(dx * std::cos(slope_angle) + dz * std::sin(slope_angle) - avg, 2);
    }
    // Calculate Standard Deviation
    return std::sqrt(var / cloud->points.size());
}


// Callback function
void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input) {
    // --- IF ADD DOWNSAMPLE, UNCOMMENT THE FOLLOWING CODE ---//

    // Convert ROS message to PCL point cloud
    PointCloud::Ptr cloud(new PointCloud);
    pcl::fromROSMsg(*input, *cloud);
    // Downsample the point cloud using VoxelGrid filter
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    PointCloud::Ptr cloud_filtered(new PointCloud);
    vg.setInputCloud(cloud);
    vg.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
    vg.filter(*cloud_filtered);
    // --- IF ADD DOWNSAMPLE, UNCOMMENT THE ABOVE CODE ---//

    // If No Downsample, uncomment the following code
    // PointCloud::Ptr cloud_filtered(new PointCloud);
    // pcl::fromROSMsg(*input, *cloud_filtered);
    // If No Downsample, uncomment the above code

    PointCloud::Ptr filtered_points(new PointCloud);
    for (const auto& point : cloud_filtered->points) {
        double dx = point.x;
        double dy = point.y;
        double angle = std::atan2(dy, dx);

        if (std::abs(angle - heading_angle_rad) < angle_threshold_rad && dx*dx+dy*dy < dist * dist) {
            filtered_points->points.push_back(point);
        }
    }

    // Check the average z value of the filtered points
    if (!filtered_points->points.empty()) {
        double slope_likelihood_value = slope_likelihood(filtered_points, M_PI/9);
        ROS_INFO("Slope likelihood: %f", slope_likelihood_value);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "slope_detection_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/3Dlidar", 1, pointCloudCallback);
    ros::spin();

    return 0;
}