#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>

class LaserScanProcessor
{
public:
    LaserScanProcessor(float angle_min, float angle_max,std::string subscribed_scan, std::string new_scan) : angle_min_(angle_min), angle_max_(angle_max)
    {
        pub_ = nh_.advertise<sensor_msgs::LaserScan>(new_scan, 10);
        sub_ = nh_.subscribe(subscribed_scan, 10, &LaserScanProcessor::callback, this);
    }

    void callback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        sensor_msgs::LaserScan filtered_scan;
        filtered_scan.header = msg->header;
        filtered_scan.angle_min = msg->angle_min;
        filtered_scan.angle_max = msg->angle_max;
        filtered_scan.angle_increment = msg->angle_increment;
        filtered_scan.time_increment = msg->time_increment;
        filtered_scan.scan_time = msg->scan_time;
        filtered_scan.range_min = msg->range_min;
        filtered_scan.range_max = msg->range_max;

        float angle = msg->angle_min;
        for (const auto& r : msg->ranges)
        {
            if (angle_min_ <= angle && angle <= angle_max_ && !std::isnan(r))
            {
                filtered_scan.ranges.push_back(r);
            }
            else
            {
                filtered_scan.ranges.push_back(std::numeric_limits<float>::infinity());
            }
            angle += msg->angle_increment;
        }

        pub_.publish(filtered_scan);
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    float angle_min_;
    float angle_max_;
};

int main(int argc, char** argv)
{
    std::string node_name = "lidar_filter";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    std::string subscribed_scan;
    if (!nh.getParam("/"+node_name+"/subscribed_scan", subscribed_scan))
    {
        ROS_ERROR("Failed to retrieve parameter 'subscribed_scan'");
        return -1;
    }
    std::string new_scan;
    if (!nh.getParam("/"+node_name+"/new_scan", new_scan))
    {
        ROS_ERROR("Failed to retrieve parameter 'new_scan'");
        return -1;
    }
    double min_angle;
    if (!nh.getParam("/"+node_name+"/min_angle", min_angle))
    {
        ROS_ERROR("Failed to retrieve parameter 'min_angle'");
        return -1;
    }
    double max_angle;
    if (!nh.getParam("/"+node_name+"/max_angle", max_angle))
    {
        ROS_ERROR("Failed to retrieve parameter 'max_angle'");
        return -1;
    }

    LaserScanProcessor processor(min_angle,max_angle,subscribed_scan,new_scan);
    ros::spin();
    return 0;
}