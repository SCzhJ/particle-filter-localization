#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include "bot_sim/EKF.h"
#include <thread>
#include <sensor_msgs/Imu.h>
#include <iostream>
#include <tf2/LinearMath/Matrix3x3.h>
#include <visualization_msgs/Marker.h>
#include <deque>

const float PI = 3.14159265358979323846;

std::string v_frame;
std::string g_frame;

std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster;
std::unique_ptr<Estimator> estimator;

tf2::Quaternion qekf1;
tf2::Quaternion qekf2;
tf2::Quaternion qpl;

std::deque<sensor_msgs::Imu::ConstPtr> imu_buffer;
double time_window = 0.1;  // 窗口

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
    qpl = tf2::Quaternion(msg->pose.pose.orientation.x, 
                            msg->pose.pose.orientation.y,
                            msg->pose.pose.orientation.z, 
                            msg->pose.pose.orientation.w);
    // Create a TransformBroadcaster object
    tf2_ros::TransformBroadcaster tfb;

    // Broadcast the transform
    // tfb.sendTransform(transformStamped);
}

void chatterCallback(const sensor_msgs::Imu::ConstPtr &imu_ptr) {
    auto cur_timestamp = imu_ptr->header.stamp.toSec();
    // std::cout<<"cur_timestamp: "<<cur_timestamp<<std::endl;
    // 将新的IMU数据添加到缓冲区
    imu_buffer.push_back(imu_ptr);

    // 移除超出时间窗口的旧数据
    while (!imu_buffer.empty() && imu_buffer.front()->header.stamp.toSec() < cur_timestamp - time_window) {
        imu_buffer.pop_front();
    }

    //average imu data
    Eigen::Vector3d avg_angular_velocity(0, 0, 0);
    Eigen::Vector3d avg_linear_acceleration(0, 0, 0);
    for (const auto& imu : imu_buffer) {
        avg_angular_velocity += Eigen::Vector3d(imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z);
        avg_linear_acceleration += Eigen::Vector3d(imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z);
    }
    avg_angular_velocity /= imu_buffer.size();
    avg_linear_acceleration /= imu_buffer.size();

    // auto pose = estimator->EstimatePose(
    //     cur_timestamp,
    //     {imu_ptr->angular_velocity.x,    imu_ptr->angular_velocity.y,    imu_ptr->angular_velocity.z},
    //     {imu_ptr->linear_acceleration.x, imu_ptr->linear_acceleration.y, imu_ptr->linear_acceleration.z}
    //     );
    // std::cout<<"avg_angular_velocity: "<<avg_angular_velocity.x()<<" "<<avg_angular_velocity.y()<<" "<<avg_angular_velocity.z()<<std::endl;

    auto pose = estimator->EstimatePose(
        cur_timestamp,
        {avg_angular_velocity.x(), avg_angular_velocity.y(), avg_angular_velocity.z()},
        {avg_linear_acceleration.x(), avg_linear_acceleration.y(), avg_linear_acceleration.z()}
    );

     qekf1 = tf2::Quaternion(pose.x(), pose.y(), pose.z(), pose.w());
    
    tf2::Matrix3x3 m(qekf1);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // std::cout<<"Quaternion: "<<qekf1.x()<<" "<<qekf1.y()<<" "<<qekf1.z()<<" "<<qekf1.w()<<std::endl;

    // std::cout << "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl;
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
    ros::Subscriber subimu1 = nh.subscribe("/livox/imu_192_168_1_141", 1000, chatterCallback);
    // imu数据融合
    // ros::Subscriber subimu2 = nh.subscribe("/imu/data2", 1000, chatterCallback);
    // q_integrate = Quaternion_S_lerp(qekf1,qekf2);

    //test
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("imu_arrow", 10);

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
    auto q = qx * qz;
    // auto q = qx * qz * q_integrate;
    transformStamped1.transform.rotation.x = q.x();
    transformStamped1.transform.rotation.y = q.y();
    transformStamped1.transform.rotation.z = q.z();
    transformStamped1.transform.rotation.w = q.w();

    ros::Rate rate(100.0);  
    while (nh.ok()){
        transformStamped1.header.stamp = ros::Time::now();
        // broadcaster.sendTransform(transformStamped1);
        //---------------------------------test---------------------------------
        
        visualization_msgs::Marker marker;
        marker.header.frame_id = "gimbal_frame";
        marker.header.stamp = ros::Time::now();

        marker.ns = "my_namespace";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;

        marker.pose.orientation.x = qekf1.x();
        marker.pose.orientation.y = qekf1.y();
        marker.pose.orientation.z = qekf1.z();
        marker.pose.orientation.w = qekf1.w();
        //  marker.pose.orientation.x = qpl.x();
        // marker.pose.orientation.y = qpl.y();
        // marker.pose.orientation.z = qpl.z();
        // marker.pose.orientation.w = qpl.w();

        marker.scale.x = 1.0;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;

        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        // Publish the marker
        marker_pub.publish(marker);
        //---------------------------------test---------------------------------
        
        ros::spinOnce();
        rate.sleep();
    }
    // ros::spinOnce();
    return 0;
}
