#include "bot_sim/dbscan_basic_bfs.h"

ros::Publisher grid_pub;
double robot_pos_x = 0.0, robot_pos_y = 0.0, robot_pos_z = 0.0, 
        robot_quat_x = 0.0, robot_quat_y = 0.0, robot_quat_z = 0.0, robot_quat_w = 0.0;
double map_pos_x = 0.0, map_pos_y = 0.0, map_pos_z = 0.0, 
        map_quat_x = 0.0, map_quat_y = 0.0, map_quat_z = 0.0, map_quat_w = 0.0;

// following get by ros param
// int obstacle_enlargement=4;
// int MAXN=200;
double epsilon = 0.6; //聚类的阈值
int minPts = 5; //聚类的最小点数

int max_radar_x = -1e8, max_radar_y = -1e8, min_radar_x = 1e8, min_radar_y = 1e8;


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

void msgs_to_grid(const sensor_msgs::LaserScan::ConstPtr& msg)
{   
    // 声明grid变量
    nav_msgs::OccupancyGrid grid;

    // 设置占用网格的基本属性
    grid.header.frame_id = "laser";// 调试雷达时改为"odom'->"laser"
    grid.info.resolution = 0.05;  // 网格的分辨率为0.05米
    grid.info.width = MAXN;  // 网格的宽度为500个单元 uint32
    grid.info.height = MAXN;  // 网格的高度为500个单元 uint32

    // 初始化占用网格的数据
    grid.data.resize(grid.info.width * grid.info.height, -1);
    // 转换数据类型 
    float x_origin = -static_cast<float>(grid.info.width * grid.info.resolution / 2.0);
    float y_origin = -static_cast<float>(grid.info.height * grid.info.resolution / 2.0);
    grid.info.origin.position.x = robot_pos_x+x_origin;// odom
    grid.info.origin.position.y = robot_pos_y+y_origin;// odom
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
    int data_size = msg->ranges.size();
    std::vector<std::vector<float>> laser_data(data_size, std::vector<float>(3, -10000));
    // 将激光扫描数据转换为占用网格

    //dbscan
    std::vector<dbscan_Point> dbscan_data;
    for(size_t i = 0; i < msg->ranges.size(); ++i){
        dbscan_Point p;
        // 计算激光点的坐标(极坐标系转换为笛卡尔坐标系)
        double temp_distance = msg->ranges[i];
        double angle = msg->angle_min + i * msg->angle_increment;
        p.x= cos(angle) * temp_distance;
        p.y= sin(angle) * temp_distance;
        dbscan_data.push_back(p);
    }
    int cnt=0;
    dbscan(dbscan_data, epsilon, minPts, cnt);

        // 将笛卡尔坐标转换为占用网格的索引 原点在网格的中心
    for(size_t i = 0; i < msg->ranges.size(); ++i){
        dbscan_data[i].x =(dbscan_data[i].x - x_origin) / grid.info.resolution;
        dbscan_data[i].y =(dbscan_data[i].y - y_origin) / grid.info.resolution;
    }
		
		//膨胀
    bfs(dbscan_data,grid_vector,grid.info.width,grid.info.height);

    
    for(size_t i = 0; i < msg->ranges.size(); ++i)
    {   
        float temp_distance = msg->ranges[i];
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
        laser_data[i][0] = grid_x;
        laser_data[i][1] = grid_y;
        laser_data[i][2] = temp_distance / grid.info.resolution;//距离转换为网格距离
        std::cout<<i<<" x:"<<grid_x<<" y:"<<grid_y<<" id:"<<dbscan_data[i].cluster<<"\n";
    }

    for (int i = 0; i < grid.info.height; ++i) {
        for (int j = 0; j < grid.info.width; ++j) {
            int index = i * grid.info.width + j;
						//阈值设置
            if (grid_vector[i][j] >= 70) grid.data[index] = 100;
        }
    }
    

    grid_pub.publish(grid);
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
    std::string node_name = "scan_2_node";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh; // 句柄 创建了一个NodeHandle对象 提供了c++与ROS系统交互的接口

    if (!ros::param::get("/" + node_name + "/obstacle_enlargement", obstacle_enlargement)) {
        ROS_ERROR("Failed to get param 'obstacle_enlargement'");
    }
    else{
        ROS_INFO("Got param 'obstacle_enlargement': %d", obstacle_enlargement);
    }

    if (!ros::param::get("/" + node_name + "/MAXN", MAXN)) {
        ROS_ERROR("Failed to get param 'MAXN'");
    }
    else{
        ROS_INFO("Got param 'MAXN': %d", MAXN);
    }

    grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("grid", 1); // (Topic Name, Queue Size)
    
    ros::Subscriber sub_1 = nh.subscribe("/scan", 1, msgs_to_grid); // 订阅sensor_msgs/LaserScan 并转换(Topic Name, Queue Size, Callback Function)
    ros::Subscriber sub_2 = nh.subscribe("odom", 10, &odom_callback);//订阅mav_msgs/Odometry
    
    // ros::Timer timer = nh.createTimer(ros::Duration(0.1), publish_transform);  // 每隔0.1秒发布一次坐标变换
    ros::Rate rate(100);
    ros::spin();

    return 0;
}
