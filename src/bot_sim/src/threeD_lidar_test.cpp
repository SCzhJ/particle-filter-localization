#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>
#include "livox_ros_driver2/CustomMsg.h"
// Include for testing
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <bot_sim/testConfig.h>


double first_RADIUS;
double second_RADIUS, slope, max_height, start_height;
void callback(bot_sim::testConfig &config, uint32_t level) {
    first_RADIUS=config.first_RADIUS;
    second_RADIUS=config.second_RADIUS;
    slope=config.slope;
    max_height=config.max_height;
    start_height=config.start_height;

}
bool get_msg = 0;
std::string base_frame;
std::string laser_frame;
std::string scan_topic_left;
std::string scan_topic_right;
std::string new_scan_topic;
ros::Publisher pub;
livox_ros_driver2::CustomMsg scan_copy_left;
livox_ros_driver2::CustomMsg scan_copy_right;
geometry_msgs::TransformStamped transformStamped;
void scanCallback_left(const livox_ros_driver2::CustomMsg &scan)
{
    scan_copy_left = scan;
    get_msg = 1;
}
void scanCallback_right(const livox_ros_driver2::CustomMsg &scan)
{
    scan_copy_right = scan;
    get_msg = 1;
}
double max_dis=0;
std::vector<double> distances;
bool satisfied(double nx, double ny, double z){
    if(nx*nx+ny*ny <= first_RADIUS*first_RADIUS){
        // if(z>0.1)printf("x: %f, y: %f, z: %f, dis: %f\n", nx, ny, z, nx*nx+ny*ny);
        return 0;
    }
    if(nx*nx+ny*ny <= second_RADIUS*second_RADIUS){
        // printf("x: %f, y: %f, z: %f, dis: %f\n", nx, ny, z, nx*nx+ny*ny);
        if(0.3<=z&&z<=0.35){
            distances.push_back(nx*nx+ny*ny);
            max_dis=std::max(max_dis,nx*nx+ny*ny);
            // printf("x: %f, y: %f, z: %f, dis: %f\n", nx, ny, z, nx*nx+ny*ny);
        }
        return z<=start_height;
    }
    double dis=sqrt(nx*nx+ny*ny)-second_RADIUS;
    return z<=std::min(max_height,dis*slope+start_height);
}

int main(int argc, char **argv)
{
    std::string node_name = "threeD_lidar_test";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    dynamic_reconfigure::Server<bot_sim::testConfig> server;
    dynamic_reconfigure::Server<bot_sim::testConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    if (!nh.getParam("/" + node_name + "/base_frame", base_frame))
    {
        ROS_ERROR("Failed to retrieve parameter 'base_frame'");
        return -1;
    }
    if (!nh.getParam("/" + node_name + "/laser_frame", laser_frame))
    {
        ROS_ERROR("Failed to retrieve parameter 'laser_frame'");
        return -1;
    }
    if (!nh.getParam("/" + node_name + "/scan_topic_left", scan_topic_left))
    {
        ROS_ERROR("Failed to retrieve parameter 'scan_topic_left'");
        return -1;
    }
    if (!nh.getParam("/" + node_name + "/scan_topic_right", scan_topic_right))
    {
        ROS_ERROR("Failed to retrieve parameter 'scan_topic_right'");
        return -1;
    }
    if (!nh.getParam("/" + node_name + "/new_scan_topic", new_scan_topic))
    {
        ROS_ERROR("Failed to retrieve parameter 'new_scan_topic'");
        return -1;
    }

    ros::Subscriber sub_left = nh.subscribe(scan_topic_left, 1, scanCallback_left);
    ros::Subscriber sub_right = nh.subscribe(scan_topic_right, 1, scanCallback_right);
    pub = nh.advertise<livox_ros_driver2::CustomMsg>(new_scan_topic, 1);
    ros::Publisher pub2 = nh.advertise<sensor_msgs::PointCloud2>("test_scan", 1);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Rate rate(50.0);

    // For testing
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    // ---
    while (ros::ok())
    {
        if (!get_msg)
        {
            ros::spinOnce();
            continue;
        }
        try
        {
            transformStamped = tfBuffer.lookupTransform(base_frame, laser_frame, ros::Time(0));
        }
        catch (tf2::TransformException &ex)
        {
            std::cout << "Error: " << ex.what() << std::endl;
        }
        max_dis=0;
        distances.clear();
        auto scan_new = scan_copy_right;
        auto scan_record_left = scan_copy_left;
        auto scan_record_right = scan_copy_right;
        scan_new.points.clear();
        pcl_cloud.points.clear();
        tf2::Transform tf_transform;
        tf2::fromMsg(transformStamped.transform, tf_transform);
        for (int i = 0; i < scan_record_left.points.size(); i++)
        {
            double x = scan_record_left.points[i].x;
            double y = scan_record_left.points[i].y;
            double z = scan_record_left.points[i].z;
            tf2::Vector3 point_in(x, y, z);
            tf2::Vector3 point_out = tf_transform * point_in;
            double nx = point_out.x();
            double ny = point_out.y();
            double nz = point_out.z();
            if (satisfied(nx,ny,z))
            {
                scan_new.points.push_back(scan_record_left.points[i]);
                pcl_cloud.points.push_back(pcl::PointXYZ(nx, ny, nz));
            }
        }
        for (int i = 0; i < scan_record_right.points.size(); i++)
        {
            double x = scan_record_right.points[i].x;
            double y = scan_record_right.points[i].y;
            double z = scan_record_right.points[i].z;
            tf2::Vector3 point_in(x, y, z);
            tf2::Vector3 point_out = tf_transform * point_in;
            double nx = point_out.x();
            double ny = point_out.y();
            double nz = point_out.z();
            if (satisfied(nx,ny,z))
            {
                scan_new.points.push_back(scan_record_right.points[i]);
                pcl_cloud.points.push_back(pcl::PointXYZ(nx, ny, nz));
            }
        }
        sort(distances.begin(),distances.end());
        for(int i=(int)distances.size()-1;i>=0;i--)printf("%lf ",distances[i]);
        printf("\nmax_dis: %f\n----------------------\n",max_dis);
        scan_new.header.stamp = ros::Time::now();
        scan_new.point_num = scan_new.points.size();
        pub.publish(scan_new);

        // For testing
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(pcl_cloud, output);
        output.header.frame_id = base_frame; // replace with your frame id
        output.header.stamp = ros::Time::now();
        pub2.publish(output);
        // ---
        // printf("Published new scan\n");
        get_msg = 0;
        rate.sleep();
    }
    return 0;
}
