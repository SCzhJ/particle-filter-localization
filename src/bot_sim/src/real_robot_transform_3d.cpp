#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include "EKF.h"
#include <thread>
#include <sensor_msgs/Imu.h>

const float PI = 3.14159265358979323846;

std::string v_frame;
std::string g_frame;

std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster;
std::unique_ptr<Estimator> estimator;

tf2::Quaternion qekf1;
tf2::Quaternion qekf2;

void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // Create a TransformStamped object
    geometry_msgs::TransformStamped transformStamped;

    // Set the header information
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = v_frame;
    transformStamped.child_frame_id = g_frame;

    // Set the translation to zero
    transformStamped.transform.translation.x = 0.0;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 0.0;

    // Set the rotation from the odometry message
    transformStamped.transform.rotation = msg->pose.pose.orientation;

    // Create a TransformBroadcaster object
    tf2_ros::TransformBroadcaster tfb;

    // Broadcast the transform
    tfb.sendTransform(transformStamped);
}

void chatterCallback(const sensor_msgs::Imu::ConstPtr &imu_ptr) {
    auto cur_timestamp = imu_ptr->header.stamp.toSec();

    auto pose = estimator->EstimatePose(
        cur_timestamp,
        {imu_ptr->angular_velocity.x,    imu_ptr->angular_velocity.y,    imu_ptr->angular_velocity.z},
        {imu_ptr->linear_acceleration.x, imu_ptr->linear_acceleration.y, imu_ptr->linear_acceleration.z}
        );

    tf2::Quaternion qekf1(pose.x(), pose.y(), pose.z(), pose.w());
    
}

int main(int argc, char** argv){
    estimator = std::make_unique<EkfEstimator>();
    std::string node_name = "real_robot_transform";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    std::string gimbal_frame;
    if (!nh.getParam("/"+node_name+"/gimbal_frame", gimbal_frame))
    {
        ROS_ERROR("Failed to retrieve parameter 'gimbal_frame'");
        return -1;
    }
    std::string _3DLidar_frame;
    if (!nh.getParam("/"+node_name+"/_3DLidar_frame", _3DLidar_frame))
    {
        ROS_ERROR("Failed to retrieve parameter '_3DLidar_frame'");
        return -1;
    }
    if (!nh.getParam("/"+node_name+"/g_frame", g_frame))
    {
        ROS_ERROR("Failed to retrieve parameter 'g_frame'");
        return -1;
    }
    if (!nh.getParam("/"+node_name+"/v_frame", v_frame))
    {
        ROS_ERROR("Failed to retrieve parameter 'v_frame'");
        return -1;
    }

    tf2_ros::TransformBroadcaster broadcaster;

    ros::Subscriber sub = nh.subscribe("/aft_mapped_to_init", 1000, odometryCallback);
    // topic记得改
    ros::Subscriber subimu1 = nh.subscribe("/imu/data1", 1000, chatterCallback);
    // imu数据融合
    // ros::Subscriber subimu1 = nh.subscribe("/imu/data2", 1000, chatterCallback);
    // q_integrate = Quaternion_S_lerp(qekf1,qekf2);
    geometry_msgs::TransformStamped transformStamped1;
    transformStamped1.header.frame_id = _3DLidar_frame;
    transformStamped1.child_frame_id = gimbal_frame;
    transformStamped1.transform.translation.x = -0.13388;
    transformStamped1.transform.translation.y = -0.11369;
    transformStamped1.transform.translation.z = 0.35;
    tf2::Quaternion qx;
    qx.setRPY(PI, 0, 0);
    tf2::Quaternion qz;
    qz.setRPY(0, 0, 45 * PI/180);
    auto q = qx * qz * qekf1;
    // auto q = qx * qz * q_integrate;
    transformStamped1.transform.rotation.x = q.x();
    transformStamped1.transform.rotation.y = q.y();
    transformStamped1.transform.rotation.z = q.z();
    transformStamped1.transform.rotation.w = q.w();

    ros::Rate rate(100.0);  
    while (nh.ok()){
        transformStamped1.header.stamp = ros::Time::now();
        broadcaster.sendTransform(transformStamped1);
        ros::spinOnce();
        rate.sleep();
    }
    // ros::spinOnce();
    return 0;
}
