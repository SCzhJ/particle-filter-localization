#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>
#include<iostream>
#include<queue>
#include<set>
#define INF 1e9
#define NEW 0
#define IN_LIST 1
#define OUT_LIST 2
class dstarlite{
    public:
        class Node;
        typedef Node* Nodeptr;
        void initialize_all(std::string map_topic);
        void reset_previous_dynamic_map_info();
        void reset();
        void astar_main(int start_x, int start_y, int goal_x, int goal_y);
        void astar_update(Nodeptr end_node);
        void astar_update_node(Nodeptr cur, double nf, Nodeptr succ);
        void dstar_main(std::string map_frame_name, std::string robot_frame_name);
        void dstar_update(Nodeptr end_node);
        void dstar_update_node(Nodeptr cur, double nf, Nodeptr succ);
        void dstar_update_node(Nodeptr cur, Nodeptr succ);
        void dstar_update_node(Nodeptr cur);
        void publish(double real_start_x, double real_start_y, ros::Publisher& pub, std::string map_frame_name);
        void when_receive_new_goal(geometry_msgs::PoseStamped::ConstPtr goal_pose_msg, std::string map_frame_name, std::string robot_frame_name, double real_start_x, double real_start_y);
        void when_receive_new_dynamic_map(nav_msgs::OccupancyGrid::ConstPtr dynamic_map_msg, std::string map_frame_name);
        int from_real_x_to_map_x(double x){return (x - initial_x) / resolution;}
        int from_real_y_to_map_y(double y){return (y - initial_y) / resolution;}
        class Node{
            public:
                double dis_to_goal, manhattan_dis_to_start, rhs, f_for_astar, f_for_dstar;
                bool isObstacle, isStaticObstacle;
                int x, y, list_status;
                Nodeptr succ;
                Node(int x, int y){
                    dis_to_goal = INF;
                    manhattan_dis_to_start = INF;
                    rhs = INF;
                    f_for_astar = INF;
                    this->isObstacle = false;
                    this->x = x;
                    this->y = y;
                    list_status = NEW;
                }
                Node(){}
                void clear(){
                    dis_to_goal = INF;
                    manhattan_dis_to_start = INF;
                    rhs = INF;
                    f_for_astar = INF;
                    f_for_dstar = INF;
                    isObstacle = isStaticObstacle;
                    list_status = NEW;
                }
                class Compare_in_Astar{public:bool operator()(Nodeptr a, Nodeptr b){return a->f_for_astar == b->f_for_astar?a->dis_to_goal < b->dis_to_goal : a->f_for_astar < b->f_for_astar;}};
                class Compare_in_Dstar{public:bool operator()(Nodeptr a, Nodeptr b){return a->f_for_dstar == b->f_for_dstar ? a->rhs < b->rhs : a->f_for_dstar < b->f_for_dstar;}};
        };
        int max_x, max_y;
        Nodeptr** map;
        std::set<Nodeptr, Node::Compare_in_Astar> astar_list;
        std::set<Nodeptr, Node::Compare_in_Dstar> dstar_list;
        std::queue<Nodeptr> changed_obstacle_nodes;
        int dx[8] = {1, 0, -1, 0, 1, -1, -1, 1};
        int dy[8] = {0, 1, 0, -1, 1, -1, 1, -1};
        ros::Rate* rate_for_staight_line, rate_for_diagonal_line;
        double velocity, resolution, initial_x, initial_y;
        void initialize_all(std::string map_topic){
            nav_msgs::OccupancyGrid::ConstPtr msg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(map_topic, nh);
            if (msg != nullptr){
                max_x = msg->info.width;
                max_y = msg->info.height;
                initial_x = msg->info.origin.position.x;
                initial_y = msg->info.origin.position.y;
                resolution = msg->info.resolution;
                //needcode to initialize max_x and max_y;
                map = new Nodeptr*[max_x];
                for(int i = 0; i < max_x; i++){
                    map[i] = new Nodeptr[max_y];
                    for(int j = 0; j < max_y; j++){
                        map[i][j] = new Node(i, j);
                        int index = i + j * max_x;
                        if (msg->data[index] == 100)map[i][j]->isObstacle = map[i][j]->isStaticObstacle = true;
                    }
                }
                rate_for_staight_line = new ros::Rate(20);
                rate_for_diagonal_line = new ros::Rate(20*1.414);
                //can be further developed by changing the refreshing rate
            }
            else{
                ROS_ERROR("Failed to get map");
                return;
            }
        }
        void when_receive_new_goal(geometry_msgs::PoseStamped::ConstPtr goal_pose_msg, double real_start_x, double real_start_y){
            int start_x, start_y, goal_x, goal_y;
            goal_x = from_real_x_to_map_x(goal_pose_msg->pose.position.x);
            goal_y = from_real_y_to_map_y(goal_pose_msg->pose.position.y);
            start_x = from_real_x_to_map_x(real_start_x);
            start_y = from_real_y_to_map_y(real_start_y);
            //need code to deal with goal and start
            astar_main(start_x, start_y, goal_x, goal_y);
            //after that, we will make most nodes into the list and quit status new.
        }
        void when_receive_new_dynamic_map(nav_msgs::OccupancyGrid::ConstPtr dynamic_map_msg, std::string map_frame_name){
            int x_size_for_dynamic_map = dynamic_map_msg->info.width;
            int y_size_for_dynamic_map = dynamic_map_msg->info.height;
            double resolution_for_dynamic_map = dynamic_map_msg->info.resolution;
            double initial_x_for_dynamic_map = dynamic_map_msg->info.origin.position.x;
            double initial_y_for_dynamic_map = dynamic_map_msg->info.origin.position.y;
            std::string dynamic_frame_name = dynamic_map_msg->header.frame_id;
            tf2_ros::Buffer tfBuffer;
            tf2_ros::TransformListener tfListener(tfBuffer);
            geometry_msgs::TransformStamped transformStamped;
            try {
                transformStamped = tfBuffer.lookupTransform(map_frame_name, dynamic_frame_name, ros::Time(0));
            } catch (tf2::TransformException &ex) {
                ROS_WARN("%s", ex.what());
                ros::Duration(1.0).sleep();
            }
            while(!changed_obstacle_nodes.empty()){
                Nodeptr cur = changed_obstacle_nodes.front();
                changed_obstacle_nodes.pop();
                cur->isObstacle = 0;
                dstar_update_node(cur);
            }
            for(int i = 0; i < x_size_for_dynamic_map; i++){
                for(int j = 0; j < y_size_for_dynamic_map; j++){
                    int index = i + j * x_size_for_dynamic_map;
                    // if (dynamic_map_msg->data[index] == 100){
                    double x_in_dynamic_map = i * resolution_for_dynamic_map + initial_x_for_dynamic_map;
                    double y_in_dynamic_map = j * resolution_for_dynamic_map + initial_y_for_dynamic_map;
                    tf2::vector3 dynamic_map_vector = tf2::vector3(x_in_dynamic_map, y_in_dynamic_map, 0);
                    tf2::vector3 map_vector = dynamic_map_vector * transformStamped.transform.rotation + transformStamped.transform.translation;
                    int x = from_real_x_to_map_x(map_vector.x());
                    int y = from_real_y_to_map_y(map_vector.y());
                    if(x < 0 || x >= max_x || y < 0 || y >= max_y)continue;
                    bool is_obstacle = dynamic_map_msg->data[index] == 100;
                    if(!map[x][y]->isObstacle && is_obstacle){
                        map[x][y]->isObstacle = true;
                        map[x][y]->rhs = INF;
                        map[x][y]->f_for_dstar = INF;
                        changed_obstacle_nodes.push(map[x][y]);
                    }
                }
            }
        }
        void reset(){for(int i = 0; i < max_x; i++)for(int j = 0; j < max_y; j++)map[i][j]->clear();}
        // dstarlite(){
            
        // }
        void astar_update_node(Nodeptr cur, double new_dis_to_goal, Nodeptr succ){
            if(new_dis_to_goal < cur->dis_to_goal){
                if(cur->list_status == IN_LIST)astar_list.erase(cur);
                cur->dis_to_goal = cur->rhs = new_dis_to_goal;
                cur->f_for_astar = cur->f_for_dstar= cur->dis_to_goal + cur->manhattan_dis_to_start;
                cur->succ = succ;
                astar_list.insert(cur);
                cur->list_status = IN_LIST;
            }
        }
        void astar_main(int start_x, int start_y, int goal_x, int goal_y){
            reset();
            astar_list.clear();
            dstar_list.clear();
            for(int i = 0; i < max_x; i++)for(int j = 0; j < max_y; j++)map[i][j]->manhattan_dis_to_start = abs(i - start_x) + abs(j - start_y);
            map[goal_x][goal_y]->dis_to_goal = 0;
            map[goal_x][goal_y]->f_for_astar = map[goal_x][goal_y]->f_for_dstar = map[goal_x][goal_y]->manhattan_dis_to_start;
            map[goal_x][goal_y]->rhs = 0;
            astar_list.insert(map[goal_x][goal_y]);
            map[goal_x][goal_y]->list_status = IN_LIST;
            astar_update(map[start_x][start_y]);
        }
        void astar_update(Nodeptr end_node){
            while(!astar_list.empty()){
                Nodeptr cur = *astar_list.begin();
                astar_list.erase(astar_list.begin());
                cur->list_status = OUT_LIST;
                if(cur == end_node)break;
                for(int i=0;i<8;i++){
                    int nx = cur->x + dx[i];
                    int ny = cur->y + dy[i];
                    if(nx < 0 || nx >= max_x || ny < 0 || ny >= max_y)continue;
                    if(map[nx][ny]->isObstacle)continue;
                    if(i<4)astar_update_node(map[nx][ny], cur->dis_to_goal + 1, cur);
                    else astar_update_node(map[nx][ny], cur->dis_to_goal + 1.414, cur);
                    //can be further developed by changing the distance between two nodes
                }
            }
        }
        void dstar_update_node(Nodeptr cur, double new_rhs, Nodeptr succ){
            if(new_rhs < cur->rhs){
                if(cur->list_status == IN_LIST)astar_list.erase(cur);
                cur->rhs = new_rhs;
                cur->f_for_dstar= cur->rhs + cur->manhattan_dis_to_start;
                cur->succ = succ;
                astar_list.insert(cur);
                cur->list_status = IN_LIST;
            }
        }
        void dstar_update_node(Nodeptr cur, Nodeptr succ){
            if(succ == cur->succ){
                if(cur->list_status == IN_LIST)astar_list.erase(cur);
                cur->rhs = INF;
                for(int i = 0; i < 8; i++){
                    int nx = cur->x + dx[i];
                    int ny = cur->y + dy[i];
                    if(nx < 0 || nx >= max_x || ny < 0 || ny >= max_y)continue;
                    if(map[nx][ny]->isObstacle)continue;
                    double new_rhs = i<4?map[nx][ny]->rhs + 1 : map[nx][ny]->rhs + 1.414;
                    if(new_rhs < cur->rhs){
                        cur->rhs = new_rhs;
                        cur->succ = map[nx][ny];
                    }
                }
                cur->f_for_dstar = cur->rhs + cur->manhattan_dis_to_start;
                dstar_list.insert(cur);
                cur->list_status = IN_LIST;
            }
        }
        void dstar_update_node(Nodeptr cur){
            if(cur->list_status == IN_LIST)astar_list.erase(cur);
            cur->rhs = INF;
            for(int i = 0; i < 8; i++){
                int nx = cur->x + dx[i];
                int ny = cur->y + dy[i];
                if(nx < 0 || nx >= max_x || ny < 0 || ny >= max_y)continue;
                if(map[nx][ny]->isObstacle)continue;
                double new_rhs = i<4?map[nx][ny]->rhs + 1 : map[nx][ny]->rhs + 1.414;
                if(new_rhs < cur->rhs){
                    cur->rhs = new_rhs;
                    cur->succ = map[nx][ny];
                }
            }
            cur->f_for_dstar = cur->rhs + cur->manhattan_dis_to_start;
            dstar_list.insert(cur);
            cur->list_status = IN_LIST;
        }

        void dstar_main(nav_msgs::OccupancyGrid::ConstPtr dynamic_map_msg, double real_start_x, double real_start_y){
            when_receive_new_dynamic_map(dynamic_map_msg, map_frame_name);
            int start_x = from_real_x_to_map_x(real_start_x);
            int start_y = from_real_y_to_map_y(real_start_y);
            dstar_update(map[start_x][start_y]);
        }
        void dstar_update(Nodeptr end_node){
            while(!dstar_list.empty()){
                Nodeptr cur = *dstar_list.begin();
                dstar_list.erase(dstar_list.begin());
                cur->list_status = OUT_LIST;
                // if(cur == end_node)break;
                //this line may effect the correction of the algorithm
                if(cur->rhs > cur->dis_to_goal){
                    cur->dis_to_goal = INF;
                    for(int i = 0; i < 8; i++){
                        int nx = cur->x + dx[i];
                        int ny = cur->y + dy[i];
                        if(nx < 0 || nx >= max_x || ny < 0 || ny >= max_y)continue;
                        if(map[nx][ny]->isObstacle)continue;
                        dstar_update_node(map[nx][ny], cur);
                    }
                    dstar_list.insert(cur);
                    cur->list_status = IN_LIST;                    
                }
                else if(cur->rhs < cur->dis_to_goal){
                    cur->dis_to_goal = cur->rhs;
                    for(int i = 0; i < 8; i++){
                        int nx = cur->x + dx[i];
                        int ny = cur->y + dy[i];
                        if(nx < 0 || nx >= max_x || ny < 0 || ny >= max_y)continue;
                        if(map[nx][ny]->isObstacle)continue;
                        if(i < 4) dstar_update_node(map[nx][ny], cur->dis_to_goal + 1, cur);
                        else dstar_update_node(map[nx][ny], cur->dis_to_goal + 1.414, cur);
                    }
                }
            }
        }
        void publish(double real_start_x, double real_start_y, ros::Publisher& pub, std::string map_frame_name){
            nav_msgs::Path path;
            path.header.stamp = ros::Time::now();
            path.header.frame_id = map_frame_name; 
            int start_x = from_real_x_to_map_x(real_start_x);
            int start_y = from_real_y_to_map_y(real_start_y);
            Nodeptr cur = map[start_x][start_y];
            while(cur != nullptr){
                geometry_msgs::PoseStamped pose;
                pose.pose.position.x = cur->x * resolution + initial_x;
                pose.pose.position.y = cur->y * resolution + initial_y;
                pose.pose.position.z = 0;
                path.poses.push_back(pose);
                cur = cur->succ;
            }
            pub.publish(path);
        }
};
bool get_dynamic_map_info = false;
nav_msgs::OccupancyGrid::ConstPtr dynamic_map_msg;
void record_dynamic_map_info(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    dynamic_map_msg = msg;    
    get_dynamic_map_info = true;
}
geometry_msgs::PoseStamped::ConstPtr goal_pose_msg;
bool get_goal_info = false;
void record_goal_info(const geometry_msgs::PoseStamped::ConstPtr& msg){
    goal_pose_msg = msg;
    get_goal_info = true;
}
int main(int argc, char **argv){
    std::string node_name = "dstarlite";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    std::string static_map_topic_name;
    if (!nh.getParam("map_topic_name", static_map_topic_name)) ROS_ERROR("Failed to get param 'map_topic_name'");
    
    std::string dynamic_map_topic_name;
    if (!nh.getParam("dynamic_map_topic_name", dynamic_map_topic_name)) ROS_ERROR("Failed to get param 'dynamic_map_topic_name'");
    nh.subscribe(dynamic_map_topic_name, 1, record_dynamic_map_info);

    std::string goal_topic_name;
    if (!nh.getParam("goal_topic_name", goal_topic_name)) ROS_ERROR("Failed to get param 'goal_topic_name'");
    nh.subscribe(goal_topic_name, 1, record_goal_info);

    ros::Publisher pub = nh.advertise<nav_msgs::Path>("path", 1);

    dstarlite dstar;
    dstar.initialize_all(static_map_topic_name);
    while(true){
        if(get_goal_info && get_dynamic_map_info){
            tf2_ros::Buffer tfBuffer;
            tf2_ros::TransformListener tfListener(tfBuffer);
            geometry_msgs::TransformStamped transformStamped;
            try {
                transformStamped = tfBuffer.lookupTransform(map_frame_name, robot_frame_name, ros::Time(0));
            } catch (tf2::TransformException &ex) {
                ROS_WARN("%s", ex.what());
                ros::Duration(1.0).sleep();
            }
            dstar.when_receive_new_goal(goal_pose_msg, transformStamped.transform.translation.x, transformStamped.transform.translation.y);
            dstar.dstar_main(dynamic_map_msg, transformStamped.transform.translation.x, transformStamped.transform.translation.y);
            dstar.publish(transformStamped.transform.translation.x, transformStamped.transform.translation.y);
            get_goal_info = get_dynamic_map_info = false;
        }else if(get_dynamic_map_info){
            tf2_ros::Buffer tfBuffer;
            tf2_ros::TransformListener tfListener(tfBuffer);
            geometry_msgs::TransformStamped transformStamped;
            try {
                transformStamped = tfBuffer.lookupTransform(map_frame_name, robot_frame_name, ros::Time(0));
            } catch (tf2::TransformException &ex) {
                ROS_WARN("%s", ex.what());
                ros::Duration(1.0).sleep();
            }
            dstar.dstar_main(dynamic_map_msg, transformStamped.transform.translation.x, transformStamped.transform.translation.y);
            dstar.publish(transformStamped.transform.translation.x, transformStamped.transform.translation.y, pub, map_frame_name);
            get_dynamic_map_info = false;
        }
    }
    return 0;
}