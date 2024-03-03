#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "tf_listener");

    ros::NodeHandle node;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Rate rate(10.0);
    while (node.ok()){
        geometry_msgs::TransformStamped transformStamped;
        try{
            transformStamped = tfBuffer.lookupTransform("base_link", "laser", ros::Time(0));
            ROS_INFO("Transform from 'laser' to 'base_link': Translation: x: %.2f, y: %.2f, z: %.2f; Rotation: x: %.2f, y: %.2f, z: %.2f, w: %.2f",
                             transformStamped.transform.translation.x,
                             transformStamped.transform.translation.y,
                             transformStamped.transform.translation.z,
                             transformStamped.transform.rotation.x,
                             transformStamped.transform.rotation.y,
                             transformStamped.transform.rotation.z,
                             transformStamped.transform.rotation.w);
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            // ros::Duration(1.0).sleep();
            // continue;
        }
        rate.sleep();
    }
    return 0;
};