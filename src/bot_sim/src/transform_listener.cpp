#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>

class TransformSubscriber
{
public:
    TransformSubscriber() : tfListener(tfBuffer)
    {
        target_frame = "map";
        source_frame = "virtual_frame";
    }

    void spin()
    {
        ros::Rate rate(10.0);
        while (ros::ok())
        {
            geometry_msgs::TransformStamped transformStamped;
            try
            {
                transformStamped = tfBuffer.lookupTransform(target_frame, source_frame, ros::Time(0),ros::Duration(50));
                ros::Time now = ros::Time::now();
                ros::Duration delay = now - transformStamped.header.stamp;
                ROS_INFO("Transform: [%f, %f, %f], Delay: %f seconds", transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z, delay.toSec());
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN("%s", ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }rate.sleep();
        }
    }

private:
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    std::string target_frame;
    std::string source_frame;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "transform_listener");
    TransformSubscriber transformSubscriber;
    transformSubscriber.spin();
    return 0;
}