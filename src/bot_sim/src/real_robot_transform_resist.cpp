#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
const float PI = 3.14159265358979323846;

std::string v_frame;
std::string g_frame;
std::string odom_frame;
tf2::Quaternion qpl;
bool received_msg = 0;
void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    received_msg = 1;
    // Create a TransformStamped object
    geometry_msgs::TransformStamped transformStamped;

    // Set the header information
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = g_frame;
    transformStamped.child_frame_id = v_frame;

    // Set the translation to zero
    transformStamped.transform.translation.x = 0.0;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 0.0;
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );
    // q = q.inverse();
    // 3d
    qpl = q;

    //
    // // 将四元数旋转应用到一个初始向量
    // tf2::Vector3 initial_vector(1, 0, 0);
    // tf2::Vector3 rotated_vector = tf2::quatRotate(qpl, initial_vector);

    // // 计算旋转后的向量在XY平面上的投影
    // tf2::Vector3 projected_vector(rotated_vector.x(), rotated_vector.y(), 0);

    // // 计算这个投影向量和初始向量之间的旋转
    // tf2::Quaternion q_correction;
    // q_correction.setRPY(0, 0, -atan2(projected_vector.y(), projected_vector.x()));

    // // 将这个旋转应用到原始的四元数旋转
    // qpl = qpl * q_correction;
    tf2::Matrix3x3 m(qpl);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    qpl.setRPY(roll, pitch, 0);
    qpl = qpl.inverse();
    // Set the rotation from the odometry message
    // transformStamped.transform.rotation = msg->pose.pose.orientation;
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    // Create a TransformBroadcaster object
    tf2_ros::TransformBroadcaster tfb;

    // Broadcast the transform
    tfb.sendTransform(transformStamped);
}

int main(int argc, char** argv){
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
    if (!nh.getParam("/"+node_name+"/odom_frame", odom_frame))
    {
        ROS_ERROR("Failed to retrieve parameter 'v_frame'");
        return -1;
    }

    tf2_ros::TransformBroadcaster broadcaster;

    ros::Subscriber sub = nh.subscribe("/aft_mapped_to_init", 1000, odometryCallback);

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

    ros::Rate rate(20.0);
    while (nh.ok()){
        if(received_msg){
            transformStamped1.header.stamp = ros::Time::now();
            auto q =  qpl  * qz ;
            transformStamped1.transform.rotation.x = q.x();
            transformStamped1.transform.rotation.y = q.y();
            transformStamped1.transform.rotation.z = q.z();
            transformStamped1.transform.rotation.w = q.w();
            broadcaster.sendTransform(transformStamped1);
            received_msg = 0;
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
