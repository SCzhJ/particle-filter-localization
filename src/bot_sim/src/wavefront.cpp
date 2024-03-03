#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>
#include <deque>

void wavefront(nav_msgs::OccupancyGrid& cost_map, int obs_x, int obs_y) {
    std::deque<std::pair<int, int>> queue;
    queue.push_back(std::make_pair(obs_x, obs_y));
    cost_map.data[obs_x + obs_y * cost_map.info.width] = 0;

    while (!queue.empty()) {
        std::pair<int, int> current = queue.front();
        queue.pop_front();

        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                int next_x = current.first + dx;
                int next_y = current.second + dy;

                if (next_x >= 0 && next_x < cost_map.info.width && 
                    next_y >= 0 && next_y < cost_map.info.height) {
                    if (cost_map.data[next_x + next_y * cost_map.info.width] == -1 || 
                    cost_map.data[next_x + next_y * cost_map.info.width] > 
                    cost_map.data[current.first + current.second * cost_map.info.width] + 1) {
                        cost_map.data[next_x + next_y * cost_map.info.width] = 
                        cost_map.data[current.first + current.second * cost_map.info.width] + 1;
                        queue.push_back(std::make_pair(next_x, next_y));
                    }
                }
            }
        }
    }
}

std::vector<std::pair<int,int>> laserScanToGrid(const sensor_msgs::LaserScan scan, geometry_msgs::TransformStamped transformStamped, nav_msgs::OccupancyGrid& cost_map) {
    std::vector<std::pair<int,int>> grid_obs_points;
    int x_grid, y_grid;
    tf2::Transform tf_transform;
    tf2::fromMsg(transformStamped.transform, tf_transform);
    for (int i = 0; i < scan.ranges.size(); i++) {
        double angle = scan.angle_min + i * scan.angle_increment;
        double x = scan.ranges[i] * cos(angle) - cost_map.info.origin.position.x;
        double y = scan.ranges[i] * sin(angle) - cost_map.info.origin.position.y;
        tf2::Vector3 point_in(x, y, 0.0);
        tf2::Vector3 point_out = tf_transform * point_in;

        int x_grid = point_out.x()/cost_map.info.resolution;
        int y_grid = point_out.y()/cost_map.info.resolution;
        if (x_grid >= 0 && x_grid < cost_map.info.width && y_grid >= 0 && y_grid < cost_map.info.height) {
            grid_obs_points.push_back(std::make_pair(x_grid, y_grid));
        }
    }
    return grid_obs_points;
}

sensor_msgs::LaserScan scan_record;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    scan_record = *scan;
}

void costMapPreprocess(nav_msgs::OccupancyGrid& cost_map, int threshold) {
    for (int i = 0; i < cost_map.info.width * cost_map.info.height; i++) {
        if (cost_map.data[i] < threshold && cost_map.data[i] != -1) {
            cost_map.data[i] = 100;
        }
        else if(cost_map.data[i] != -1){
            cost_map.data[i] = 0;
        }
    }
}

int main(int argc, char** argv) {
    std::string node_name = "wavefront";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    std::string base_frame;
    if (!nh.getParam("/"+node_name+"/base_frame", base_frame))
    {
        ROS_ERROR("Failed to retrieve parameter 'base_frame'");
        return -1;
    }
    std::string laser_frame;
    if (!nh.getParam("/"+node_name+"/laser_frame", laser_frame))
    {
        ROS_ERROR("Failed to retrieve parameter 'laser_frame'");
        return -1;
    }
    std::string scan_topic;
    if (!nh.getParam("/"+node_name+"/scan_topic", scan_topic))
    {
        ROS_ERROR("Failed to retrieve parameter 'scan_topic'");
        return -1;
    }
    std::string map_topic;
    if (!nh.getParam("/"+node_name+"/map_topic", map_topic))
    {
        ROS_ERROR("Failed to retrieve parameter 'map_topic'");
        return -1;
    }
    int map_width;
    if (!nh.getParam("/"+node_name+"/map_width", map_width))
    {
        ROS_ERROR("Failed to retrieve parameter 'map_width'");
        return -1;
    }
    int map_height;
    if (!nh.getParam("/"+node_name+"/map_height",map_height)){
        ROS_ERROR("Failed to retrieve parameter 'map_height'");
        return -1;
    }
    int obs_threshold;
    if (!nh.getParam("/"+node_name+"/obs_threshold", obs_threshold))
    {
        ROS_ERROR("Failed to retrieve parameter 'obs_threshold'");
        return -1;
    }
    double delta_time;
    if (!nh.getParam("/"+node_name+"/delta_time", delta_time))
    {
        ROS_ERROR("Failed to retrieve parameter 'delta_time'");
        return -1;
    }
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    geometry_msgs::TransformStamped transformStamped;

    ros::Subscriber sub = nh.subscribe(scan_topic, 1, scanCallback);
    ros::Publisher pub = nh.advertise<nav_msgs::OccupancyGrid>(map_topic, 1);

    ros::Rate rate(1/delta_time);

    nav_msgs::OccupancyGrid cost_map;
    cost_map.header.frame_id = base_frame;
    cost_map.info.resolution = 0.05;
    cost_map.info.width = map_width;
    cost_map.info.height = map_height;
    cost_map.info.origin.position.x = - map_width * cost_map.info.resolution / 2; 
    cost_map.info.origin.position.y = - map_width * cost_map.info.resolution / 2;
    cost_map.info.origin.position.z = 0;
    cost_map.info.origin.orientation.x = 0;
    cost_map.info.origin.orientation.y = 0;
    cost_map.info.origin.orientation.z = 0;
    cost_map.info.origin.orientation.w = 1;

    std::vector<std::pair<int,int>> grid_obs_points;
    cost_map.data.resize(cost_map.info.width * cost_map.info.height, -1);

    while(ros::ok()) {
        std::fill(cost_map.data.begin(), cost_map.data.end(), -1);
        try {
            transformStamped = tfBuffer.lookupTransform(base_frame, laser_frame, ros::Time(0));
        } catch (tf2::TransformException &ex) {
            continue;
        }
        grid_obs_points = laserScanToGrid(scan_record, transformStamped, cost_map);

        for (int i = 0; i < grid_obs_points.size(); i++) {
            wavefront(cost_map, grid_obs_points[i].first, grid_obs_points[i].second);
        }

        costMapPreprocess(cost_map, obs_threshold);
        pub.publish(cost_map);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}