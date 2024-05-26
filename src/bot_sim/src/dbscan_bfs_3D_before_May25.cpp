#pragma once
#include <chrono>
#include <queue>
#include "ros/ros.h"
#include <iostream>
#include <fstream>   
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Transform.h>
#include <ros/ros.h> 
#include <nav_msgs/Odometry.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <string>
#include <tf/transform_datatypes.h>
#include <deque>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
//雷达参数 theta_square = 0.000279328; //theta = 0.0167131

extern int max_radar_x;
extern int max_radar_y;
extern int min_radar_x;
extern int min_radar_y;
int obstacle_enlargement=4;
int MAXN=50;
//bfs方向数组
int dx[4] = {0, 0, 1, -1};
int dy[4] = {1, -1, 0, 0};
int dx2[4] = {1, -1, 1, -1};
int dy2[4] = {1, 1, -1, -1};

//dbscan 点
struct dbscan_Point {
    float x, y, z;
    int cluster = 0;
    bool visited = false;
    bool noise = false;
};
//整数点
struct int_Point {
    int x, y;
};

//double距离
double dbscan_distance(dbscan_Point a, dbscan_Point b) {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2));
}

//聚类函数
void dbscan(std::vector<dbscan_Point>& data, double eps, int minPts,int& clusterId) {
    clusterId=1;
    for (dbscan_Point& p : data) { 
        if (p.visited) continue;
        p.visited = true;

        std::vector<dbscan_Point*> neighbors;
        for (dbscan_Point& q : data) {
            if (dbscan_distance(p, q) <= eps) {
                neighbors.push_back(&q);
            }
        }

        if (neighbors.size() < minPts) {
            p.noise = true;
        } 
        else {
            clusterId++;
            p.cluster = clusterId;

            for (dbscan_Point* q : neighbors) {
                if (!q->visited) {
                    q->visited = true;
                    std::vector<dbscan_Point*> neighbors2;
                    for (dbscan_Point& r : data) {
                        if (dbscan_distance(*q, r) <= eps) {
                            // std::cout<<dbscan_distance(*q, r)<<"\n";
                            neighbors2.push_back(&r);
                        }
                    }
                    if (neighbors2.size() >= minPts) {
                        for (dbscan_Point* r : neighbors2) {
                            if (r->cluster == 0) {
                                r->cluster = clusterId;
                            }
                        }
                    }
                }
                if (q->cluster == 0) {
                    q->cluster = clusterId;
                }
            }
        }
    }
}

//bfs膨胀
void bfs(std::vector<dbscan_Point>& points, std::vector<std::vector<int>>& data, int width, int height, int decrease) {
    std::queue<std::pair<int_Point, int>> q;
    for (dbscan_Point& p : points) {
        if (p.noise) continue;
        int tx = static_cast<int>(p.x);
        int ty = static_cast<int>(p.y);
        if (ty < 0 || ty >= height || tx < 0 || tx >= width) continue;
        data[ty][tx] = 100;  // 初始点值为100
        q.push({{tx, ty}, 100});
    }

    while (!q.empty()) {
        int_Point p = q.front().first;
        int value = q.front().second;
        q.pop();

        for (int i = 0; i < 4; ++i) {
            int nx = p.x + dx[i];
            int ny = p.y + dy[i];
            if (nx < 0 || ny >= height || ny < 0 || nx >= width) {
                continue;
                // std::cout<<"out of range\n";
            }
            int next_value = std::max(0, int(value - decrease));
            if (data[ny][nx] < next_value) {
                data[ny][nx] = next_value;
                q.push({{nx, ny}, next_value});
            }
        }
        for (int i = 0; i < 4; i++) {
            int nx = p.x + dx2[i];
            int ny = p.y + dy2[i];
            if (nx < 0 || ny >= height || ny < 0 || nx >= width) {
                continue;
                // std::cout<<"out of range\n";
            }
            int next_value = std::max(0, int(value - decrease*1.4));
            if (data[ny][nx] < next_value) {
                data[ny][nx] = next_value;
                q.push({{nx, ny}, next_value});
            }
        }
    }
}

ros::Publisher grid_pub;
double robot_pos_x = 0.0, robot_pos_y = 0.0, robot_pos_z = 0.0, 
        robot_quat_x = 0.0, robot_quat_y = 0.0, robot_quat_z = 0.0, robot_quat_w = 0.0;
double map_pos_x = 0.0, map_pos_y = 0.0, map_pos_z = 0.0, 
        map_quat_x = 0.0, map_quat_y = 0.0, map_quat_z = 0.0, map_quat_w = 0.0;

// following get by ros param
// int obstacle_enlargement=4;
// int MAXN=200;Z
double epsilon = 0.2; //聚类的阈值
int minPts = 7; //聚类的最小点数
int dfs_decrease = 5;
int dfs_threshold = 50;
std::string frame_name = "gimbal_frame";
std::string parent_frame, child_frame;

int max_radar_x = -1e8, max_radar_y = -1e8, min_radar_x = 1e8, min_radar_y = 1e8;
// extern tf2_ros::TransformListener tfListener(tfBuffer);
geometry_msgs::TransformStamped transformStamped;
pcl::PointCloud<pcl::PointXYZ>::Ptr scan_record(new pcl::PointCloud<pcl::PointXYZ>);
bool get_scan;

void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    //获取平移信息
    robot_pos_x = msg->pose.pose.position.x;
    robot_pos_y = msg->pose.pose.position.y;
    robot_pos_z = msg->pose.pose.position.z;
    //获取旋转信息
    robot_quat_x = msg->pose.pose.orientation.x;
    robot_quat_y = msg->pose.pose.orientation.y;
    robot_quat_z = msg->pose.pose.orientation.z;
    robot_quat_w = msg->pose.pose.orientation.w;
}

void msgs_to_grid(const sensor_msgs::PointCloud2ConstPtr& msg)
{   
    pcl::fromROSMsg(*msg, *scan_record);
    get_scan=1;
    
}

void publish_transform(const ros::TimerEvent&)
{
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    // map->odom
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "odom";
    transformStamped.transform.translation.x = 0;
    transformStamped.transform.translation.y = 0;
    transformStamped.transform.translation.z = 0;
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    transformStamped.transform.rotation.x = 0;
    transformStamped.transform.rotation.y = 0;
    transformStamped.transform.rotation.z = 0;
    transformStamped.transform.rotation.w = 0;

    br.sendTransform(transformStamped);
}


int main(int argc, char **argv)
{
            printf("start");
    std::string node_name = "dbscan_bfs_3D";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh; // 句柄 创建了一个NodeHandle对象 提供了c++与ROS系统交互的接口
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    if (!ros::param::get("/" + node_name + "/epsilon", epsilon)) {
        ROS_ERROR("Failed to get param 'epsilon'");
    }
    else{
        ROS_INFO("Got param 'epsilon': %lf", epsilon);
    }
    if (!ros::param::get("/" + node_name + "/minPts", minPts)) {
        ROS_ERROR("Failed to get param 'minPts'");
    }
    else{
        ROS_INFO("Got param 'minPts': %d", minPts);
    }
    if (!ros::param::get("/" + node_name + "/dfs_decrease", dfs_decrease)) {
        ROS_ERROR("Failed to get param 'dfs_decrease'");
    }
    else{
        ROS_INFO("Got param 'dfs_decrease': %d", dfs_decrease);
    }
    if (!ros::param::get("/" + node_name + "/dfs_threshold", dfs_threshold)) {
        ROS_ERROR("Failed to get param 'dfs_threshold'");
    }
    else{
        ROS_INFO("Got param 'dfs_threshold': %d", dfs_threshold);
    }
    if (!ros::param::get("/" + node_name + "/child_frame", child_frame)) {
        ROS_ERROR("Failed to get param 'child_frame'");
    }
    else{
        ROS_INFO("Got param 'child_frame': %s", child_frame.c_str());
    }
    if (!ros::param::get("/" + node_name + "/parent_frame", parent_frame)) {
        ROS_ERROR("Failed to get param 'parent_frame'");
    }
    else{
        ROS_INFO("Got param 'parent_frame': %s", parent_frame.c_str());
    }
    frame_name=child_frame;
    grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("grid", 1); // (Topic Name, Queue Size)
    
    ros::Subscriber sub_1 = nh.subscribe("/test_scan", 1, msgs_to_grid); // 订阅sensor_msgs/LaserScan 并转换(Topic Name, Queue Size, Callback Function)
    ros::Subscriber sub_2 = nh.subscribe("odom", 10, &odom_callback);//订阅mav_msgs/Odometry
    ros::Rate rate(100);
    while(ros::ok){
        if(get_scan){
            auto start_ = std::chrono::high_resolution_clock::now();
            printf("get_scan");
            // 声明grid变量
            nav_msgs::OccupancyGrid grid;

            // 设置占用网格的基本属性
            grid.header.frame_id = frame_name;// 调试雷达时改为"odom'->"laser"
            grid.info.resolution = 0.05;  // 网格的分辨率为0.05米
            grid.info.width = MAXN;  // 网格的宽度为500个单元 uint32
            grid.info.height = MAXN;  // 网格的高度为500个单元 uint32

            // 初始化占用网格的数据
            grid.data.resize(grid.info.width * grid.info.height, 0);
            // 转换数据类型 
            float x_origin = -static_cast<float>(grid.info.width * grid.info.resolution / 2.0);
            float y_origin = -static_cast<float>(grid.info.height * grid.info.resolution / 2.0);
            grid.info.origin.position.x = x_origin;// ladar
            grid.info.origin.position.y = y_origin;// ladar
            grid.info.origin.position.z = 0.0;
            tf::Quaternion q(robot_quat_x, robot_quat_y, robot_quat_z, robot_quat_w);
            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            // 创建一个二维数组
            std::vector<std::vector<int>> grid_vector(grid.info.height, std::vector<int>(grid.info.width, 0));
            // 创建储存雷达数据的二维数组
            int data_size = scan_record->points.size();
            // 将激光扫描数据转换为占用网格

            //dbscan
            tf2::Transform tf_transform;
            tf2::fromMsg(transformStamped.transform, tf_transform);
            try{
                transformStamped = tfBuffer.lookupTransform(child_frame, parent_frame, ros::Time(0));
            }
            catch (tf2::TransformException &ex){
                std::cout << "Error: " << ex.what() << std::endl;
            }
            std::vector<dbscan_Point> dbscan_data;
            for(size_t i = 0; i < scan_record->points.size(); ++i){
                dbscan_Point p;
                // 计算激光点的坐标(极坐标系转换为笛卡尔坐标系)
                p.x= scan_record->points[i].x;
                p.y= scan_record->points[i].y;
                p.z= scan_record->points[i].z;
                tf2::Vector3 point_in(p.x, p.y, p.z);
                tf2::Vector3 point_out = tf_transform * point_in;
                p.x=point_out[0];
                p.y=point_out[1];
                p.z=point_out[2];
                if(p.x - x_origin > grid.info.width * grid.info.resolution || p.x - x_origin < 0 || p.y - y_origin < 0 ||  p.y - y_origin > grid.info.height * grid.info.resolution) continue;
                // std::cout<<p.x<<' '<<p.y<<"\n";
                dbscan_data.push_back(p);
            }
            int cnt=0;
            dbscan(dbscan_data, epsilon, minPts, cnt);

                // 将笛卡尔坐标转换为占用网格的索引 原点在网格的中心
            for(size_t i = 0; i < dbscan_data.size(); ++i){
                dbscan_data[i].x =(dbscan_data[i].x - x_origin) / grid.info.resolution;
                dbscan_data[i].y =(dbscan_data[i].y - y_origin) / grid.info.resolution;
            }
                
                //膨胀
            bfs(dbscan_data,grid_vector,grid.info.width,grid.info.height, dfs_decrease);

            
            for(size_t i = 0; i < dbscan_data.size(); ++i)
            {   
                int grid_x = dbscan_data[i].x;
                int grid_y = dbscan_data[i].y;
                // std::cout<<i<<" x:"<<x<<" y:"<<y<<"\n";
                // 将笛卡尔坐标转换为占用网格的索引 原点在网格的中心
                // int grid_x =(x - x_origin) / grid.info.resolution;
                // int grid_y =(y - y_origin) / grid.info.resolution;
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
                if (!dbscan_data[i].noise) grid_vector[grid_y][grid_x] = 100;
                else if (dbscan_data[i].cluster == 0) grid_vector[grid_y][grid_x] = -1;
                else grid_vector[grid_y][grid_x] = 50;
                // std::cout<<i<<" x:"<<grid_x<<" y:"<<grid_y<<" id:"<<dbscan_data[i].cluster<<"\n";
            }

            for (int i = 0; i < grid.info.height; ++i) {
                for (int j = 0; j < grid.info.width; ++j) {
                    int index = i * grid.info.width + j;

                                //阈值设置
                    if (grid_vector[i][j] >= dfs_threshold) grid.data[index] = 100;
                }
            }
            
            auto end_ = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> diff = end_-start_;

            std::cout << "Time difference: " << diff.count() << " s\n";
            grid_pub.publish(grid);
            get_scan=0;
        }
        ros::spinOnce();    
        rate.sleep();
    }
    // ros::Timer timer = nh.createTimer(ros::Duration(0.1), publish_transform);  // 每隔0.1秒发布一次坐标变换
    ros::spin();

    return 0;
}
