#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "robot_tf_publisher");
    ros::NodeHandle n;

    tf2_ros::TransformBroadcaster broadcaster;

    geometry_msgs::TransformStamped transformStamped1;
    transformStamped1.header.frame_id = "aft_mapped";
    transformStamped1.child_frame_id = "base_footprint";
    transformStamped1.transform.translation.x = 0.0;
    transformStamped1.transform.translation.y = -0.234;
    transformStamped1.transform.translation.z = 0.0;
    transformStamped1.transform.rotation.x = 0;
    transformStamped1.transform.rotation.y = 0;
    transformStamped1.transform.rotation.z = sqrt(2)/2;
    transformStamped1.transform.rotation.w = sqrt(2)/2;

    geometry_msgs::TransformStamped transformStamped2;
    transformStamped2.header.frame_id = "base_footprint";
    transformStamped2.child_frame_id = "laser";
    transformStamped2.transform.translation.x = 0.234;
    transformStamped2.transform.translation.y = 0.0;
    transformStamped2.transform.translation.z = 0.0;
    transformStamped2.transform.rotation.x = 0;
    transformStamped2.transform.rotation.y = 0;
    transformStamped2.transform.rotation.z = 1;
    transformStamped2.transform.rotation.w = 0;

    ros::Rate rate(20.0); // 10 Hz
    while (n.ok()){
        transformStamped1.header.stamp = ros::Time::now();
        transformStamped2.header.stamp = ros::Time::now();

        broadcaster.sendTransform(transformStamped1);
        broadcaster.sendTransform(transformStamped2);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}