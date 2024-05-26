#include "ros/ros.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <string>
#include "bot_sim/navi_basic_func.h"

ros::Publisher grid_pub;
tf::StampedTransform transform;
double map_pos_x = 0.0, map_pos_y = 0.0, map_pos_z = 0.0, 
        map_quat_x = 0.0, map_quat_y = 0.0, map_quat_z = 0.0, map_quat_w = 0.0;

// following get by ros param
int obstacle_enlargement, MAXN, threshold;
std::string scan_topic, obstacle_map_topic, base_frame, laser_frame;

int max_radar_x = -1e8, max_radar_y = -1e8, min_radar_x = 1e8, min_radar_y = 1e8;


// 声明grid变量
nav_msgs::OccupancyGrid grid;
float x_origin, y_origin;


//测试
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    std::vector<float> ranges=msg->ranges;
    std::cout<< ranges.size()<<" ";
    std::cout<< msg->header.stamp<<" "; //time
    std::cout<< msg->header.frame_id<<" ";
    std::cout<< msg->angle_min<<" "; 
    std::cout<< msg->angle_max<<" ";
	std::cout<< msg->angle_increment<<" ";
	std::cout<< msg->time_increment<<" ";
	std::cout<< "dis_ranges:"<<  msg->range_min<<" ";
	std::cout<< msg->range_max<<" ";
    std::cout<<"\n";
}

void msgs_to_grid(const sensor_msgs::LaserScan::ConstPtr& msg)
{   
    // 创建一个二维数组
    std::vector<std::vector<int>> grid_vector(grid.info.height, std::vector<int>(grid.info.width, 0));
    // 创建储存雷达数据的二维数组
    int data_size = msg->ranges.size();
    std::vector<std::vector<float>> laser_data(data_size, std::vector<float>(3, -10000));
    // 将激光扫描数据转换为占用网格
    for(size_t i = 0; i < msg->ranges.size(); ++i)
    {   
        float temp_distance = msg->ranges[i];
        // 计算激光点的坐标(极坐标系转换为笛卡尔坐标系)
        float angle = msg->angle_min + i * msg->angle_increment;
        float x = cos(angle) * temp_distance ;
        float y = sin(angle) * temp_distance ;
        // 将激光点的坐标转换为base_footprint坐标系
        tf::Point obstacle_laser_frame(x, y, 0);
        tf::Point obstacle_base_footprint_frame = transform * obstacle_laser_frame;

        // 将笛卡尔坐标转换为占用网格的索引 原点在网格的中心
        int grid_x =(obstacle_base_footprint_frame.x() - x_origin) / grid.info.resolution;
        int grid_y =(obstacle_base_footprint_frame.y() - y_origin) / grid.info.resolution;
        max_radar_x = max_radar_x > grid_x ? max_radar_x : grid_x;
        max_radar_y = max_radar_y > grid_y ? max_radar_y : grid_y;
        min_radar_x = min_radar_x < grid_x ? min_radar_x : grid_x;
        min_radar_y = min_radar_y < grid_y ? min_radar_y : grid_y;
        // 剔除超过地图范围的点
        if (grid_x < 0 || grid_x >= grid.info.width || grid_y < 0 || grid_y >= grid.info.height) {
            data_size--;
            continue;
        }
        // 计算二维坐标的一维索引
        grid_vector[grid_y][grid_x] = 100;
        laser_data[i][0] = grid_x;
        laser_data[i][1] = grid_y;
        laser_data[i][2] = temp_distance / grid.info.resolution;//距离转换为网格距离
        // std::cout<<i<<" x:"<<grid_x<<" y:"<<grid_y<<"\n";
    }
    // 连线法填充
    filling_func(laser_data, grid_vector, data_size);
    for (int i = 0; i < grid.info.height; ++i) {
        for (int j = 0; j < grid.info.width; ++j) {
            int index = i * grid.info.width + j;
            grid.data[index]=grid_vector[i][j];
        }
    }
    
    //IDW插值
    std::vector<std::vector<int>> grid_idw_vector(grid.info.height, std::vector<int>(grid.info.width, 0));
    int temp_grid_data=0;
    for (int i = 0; i < grid.info.height; ++i) {
        for (int j = 0; j < grid.info.width; ++j) {
            int index = i * grid.info.width + j;
            if (grid_vector[i][j] == 100) {grid_idw_vector[i][j]=100;grid.data[index]=100;continue;}
            grid_idw_vector[i][j] = inverse_distance_weighting(grid_vector, i, j, obstacle_enlargement);
            temp_grid_data=grid_idw_vector[i][j];
            grid.data[index] = temp_grid_data> threshold ? 100 : 0;
        }     
    }

    grid_pub.publish(grid);
}

void logParam(const std::string& node_name, const std::string& param_name, const std::string& param_value);
void logParam(const std::string& node_name, const std::string& param_name, const int& param_value);
template <typename T>
bool getParam(const std::string& node_name, const std::string& param_name, T& param_value);

void initializeGrid();

int main(int argc, char **argv)
{
    std::string node_name = "scan_2_node";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh; // 句柄 创建了一个NodeHandle对象 提供了c++与ROS系统交互的接口

    getParam(node_name, "MAXN", MAXN);
    getParam(node_name, "obstacle_enlargement", obstacle_enlargement);
    getParam(node_name, "scan_topic", scan_topic);
    getParam(node_name, "obstacle_map_topic", obstacle_map_topic);
    getParam(node_name, "base_frame", base_frame);
    getParam(node_name, "laser_frame", laser_frame);
    getParam(node_name, "threshold", threshold);

    initializeGrid();

    // // Create a TransformListener
    // tf::TransformListener listener;

    // // Wait for the transform to become available
    // ros::Time now = ros::Time::now();
    // listener.waitForTransform(base_frame, laser_frame, now, ros::Duration(10.0));

    // // Get the transform
    // listener.lookupTransform(base_frame, laser_frame, now, transform);

    // // // Now you can use the transform to convert coordinates from the laser frame to the base_footprint frame
    // // for (size_t i = 0; i < obstacles.size(); ++i) {
    // //     tf::Point obstacle_laser_frame(obstacles[i].x, obstacles[i].y, obstacles[i].z);
    // //     tf::Point obstacle_base_footprint_frame = transform * obstacle_laser_frame;
    // // }
    // Create a TransformListener object
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    // Wait for the transform to become available
    ros::Rate rate(10.0);
    bool transformAvailable = false;
    while (ros::ok() && !transformAvailable)
    {
        try
        {
            geometry_msgs::TransformStamped transformStamped;
            transformStamped = tfBuffer.lookupTransform(base_frame, laser_frame, ros::Time(0));
            tf::transformStampedMsgToTF(transformStamped, transform);
            transformAvailable = true;  // Set the flag to stop the loop
            ROS_INFO("Got transform");
        }
        catch (tf2::TransformException& ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
        rate.sleep();
    }


    grid_pub = nh.advertise<nav_msgs::OccupancyGrid>(obstacle_map_topic, 1); // (Topic Name, Queue Size)
    
    ros::Subscriber sub_1 = nh.subscribe(scan_topic, 1, msgs_to_grid); // 订阅sensor_msgs/LaserScan 并转换(Topic Name, Queue Size, Callback Function)

    ros::spin();

    return 0;
}

void logParam(const std::string& node_name, const std::string& param_name, const std::string& param_value) {
    ROS_INFO("Got param '%s': %s", param_name.c_str(), param_value.c_str());
}
void logParam(const std::string& node_name, const std::string& param_name, const int& param_value) {
    ROS_INFO("Got param '%s': %d", param_name.c_str(), param_value);
}
template <typename T>
bool getParam(const std::string& node_name, const std::string& param_name, T& param_value) {
    if (!ros::param::get("/" + node_name + "/" + param_name, param_value)) {
        ROS_ERROR("Failed to get param '%s'", param_name.c_str());
        return false;
    }
    else{
        logParam(node_name, param_name, param_value);
        return true;
    }
}

void initializeGrid(){
    // 设置占用网格的基本属性
    grid.header.frame_id = base_frame;// 调试雷达时改为"odom'->"laser"
    grid.info.resolution = 0.05;  // 网格的分辨率为0.05米
    grid.info.width = MAXN;  // 网格的宽度为500个单元 uint32
    grid.info.height = MAXN;  // 网格的高度为500个单元 uint32
    // 初始化占用网格的数据
    grid.data.resize(grid.info.width * grid.info.height, -1);
    // 转换数据类型 
    x_origin = -static_cast<float>(grid.info.width * grid.info.resolution / 2.0);
    y_origin = -static_cast<float>(grid.info.height * grid.info.resolution / 2.0);
    grid.info.origin.position.x = x_origin;// ladar
    grid.info.origin.position.y = y_origin;// ladar
    grid.info.origin.position.z = 0.0;
}