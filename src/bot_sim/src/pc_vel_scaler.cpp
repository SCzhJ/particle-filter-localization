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
double z_threshold = -0.38;
double voxel_leaf_size = 0.05;  // Leaf size for downsampling
double dist = 2.2;
double variance_threshold = 0.15;

// Moving average
int moving_avg_count = 5;
std::vector<double> moving_avg_z = std::vector<double>(moving_avg_count, 0.0);
std::vector<double> moving_avg_var = std::vector<double>(moving_avg_count, 0.0);
double moving_z = 0.0;
double moving_var = 0.0;

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
        double avg_z = 0.0;
        for (const auto& point : filtered_points->points) {
            avg_z += point.z;
        }
        avg_z /= filtered_points->points.size();


        // detect the variance of distance on the x-y plane of the points
        double avg_x = 0.0;
        double avg_y = 0.0;
        for (const auto& point : filtered_points->points) {
            avg_x += point.x;
            avg_y += point.y;
        }
        avg_x /= filtered_points->points.size();
        avg_y /= filtered_points->points.size();
        double variance = 0.0;
        for (const auto& point : filtered_points->points) {
            variance += (point.x - avg_x) * (point.x - avg_x) + (point.y - avg_y) * (point.y - avg_y);
        }
        variance /= filtered_points->points.size();

        moving_avg_z.push_back(avg_z); moving_avg_z.erase(moving_avg_z.begin());
        moving_avg_var.push_back(variance); moving_avg_var.erase(moving_avg_var.begin());
        // get average of moving_avg_z and moving_avg_var
        moving_z = 0.0; moving_var = 0.0;
        for (const auto& z : moving_avg_z) {
            moving_z += z;
        }
        for (const auto& var : moving_avg_var) {
            moving_var += var;
        }
        moving_z /= moving_avg_z.size();
        moving_var /= moving_avg_var.size();

        // ROS_INFO("Num Points: %ld", filtered_points->points.size());
        // ROS_INFO("Average Z: %f", avg_z);
        // ROS_INFO("Variance: %f", variance);
        if(moving_z > z_threshold and moving_var > variance_threshold){
            ROS_INFO("Slope Detected");

        }
        else{
            ROS_INFO("No Slope Detected");
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "slope_detection_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/3Dlidar", 1, pointCloudCallback);
    ros::spin();

    return 0;
}