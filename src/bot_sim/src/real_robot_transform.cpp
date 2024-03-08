#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char** argv){
    std::string node_name = "real_robot_transform";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    std::string rotbase_frame;
    if (!nh.getParam("/"+node_name+"/rotbase_frame", rotbase_frame))
    {
        ROS_ERROR("Failed to retrieve parameter 'rotbase_frame'");
        return -1;
    }
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
    std::string _2DLidar_frame;
    if (!nh.getParam("/"+node_name+"/_2DLidar_frame", _2DLidar_frame))
    {
        ROS_ERROR("Failed to retrieve parameter '_2DLidar_frame'");
        return -1;
    }

    tf2_ros::TransformBroadcaster broadcaster;

    geometry_msgs::TransformStamped transformStamped1;
    transformStamped1.header.frame_id = _3DLidar_frame;
    transformStamped1.child_frame_id = gimbal_frame;
    transformStamped1.transform.translation.x = 0.0;
    transformStamped1.transform.translation.y = 0.0;
    transformStamped1.transform.translation.z = 0.0;
    tf2::Quaternion q1;
    q1.setRPY(180 * M_PI / 180.0, 0, 0);
    transformStamped1.transform.rotation.x = q1.x();
    transformStamped1.transform.rotation.y = q1.y();
    transformStamped1.transform.rotation.z = q1.z();
    transformStamped1.transform.rotation.w = q1.w();
    
    geometry_msgs::TransformStamped transformStamped2;
    transformStamped2.header.frame_id = gimbal_frame;
    transformStamped2.child_frame_id = _2DLidar_frame;
    transformStamped2.transform.translation.x = 0.0;
    transformStamped2.transform.translation.y = 0.0;
    transformStamped2.transform.translation.z = 0.0;
    transformStamped2.transform.rotation.x = 0;
    transformStamped2.transform.rotation.y = 0;
    transformStamped2.transform.rotation.z = 0;
    transformStamped2.transform.rotation.w = 1;

    ros::Rate rate(100.0);
    while (nh.ok()){
        transformStamped1.header.stamp = ros::Time::now();
        transformStamped2.header.stamp = ros::Time::now();

        broadcaster.sendTransform(transformStamped1);
        broadcaster.sendTransform(transformStamped2);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
