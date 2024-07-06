#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_broadcaster.h>

class ParticleFilter
{
public:
    ParticleFilter()
    {
        // Initialize ROS node
        ros::NodeHandle nh;

        // Subscribe to laser scan data
        laser_sub = nh.subscribe("/scan", 10, &ParticleFilter::laserCallback, this);

        // Publish particle cloud
        particles_pub = nh.advertise<geometry_msgs::PoseArray>("/particles", 1);

        // TODO: Initialize your particles here
    }

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        // TODO: Implement your particle filter update here

        // Publish particles
        geometry_msgs::PoseArray particles_msg;
        // TODO: Fill particles_msg with your particles
        particles_pub.publish(particles_msg);
    }

private:
    ros::Subscriber laser_sub;
    ros::Publisher particles_pub;
};

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "mcl");

    // Create particle filter
    ParticleFilter pf;

    // Spin
    ros::spin();

    return 0;
}