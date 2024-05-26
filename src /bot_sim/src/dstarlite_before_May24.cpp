#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>
#include<iostream>
#include<queue>
#include<stack>
#include<set>
#define INF 1e9
#define NEW 0
#define IN_LIST 1
#define OUT_LIST 2
ros::Publisher *pub_map;
class dstarlite{
    public:
        class Node;
        typedef Node* Nodeptr;
        void reset_previous_dynamic_map_info();
        void reset();
        // void astar_main(int start_x, int start_y, int goal_x, int goal_y);
        // void astar_update(Nodeptr end_node);
        // void astar_update_node(Nodeptr cur, double nf, Nodeptr succ);
        void dstar_main(nav_msgs::OccupancyGrid::ConstPtr dynamic_map_msg, double real_start_x, double real_start_y, std::string robot_frame_name, tf2_ros::Buffer& tfBuffer);
        void dstar_update(Nodeptr end_node);
        void dstar_update_node(Nodeptr cur, double nf, Nodeptr succ);
        void dstar_update_node(Nodeptr cur, Nodeptr succ);
        void dstar_update_node(Nodeptr cur);
        void publish(ros::Publisher& pub, std::string map_frame_name, ros::Publisher& cmd_vel_pub, std::string robot_frame_name, tf2_ros::Buffer& tfBuffer);
        void publish_all_map_status(ros::Publisher& pub);
        void when_receive_new_goal(geometry_msgs::PointStamped::ConstPtr goal_pose_msg, double real_start_x, double real_start_y);
        void when_receive_new_dynamic_map(nav_msgs::OccupancyGrid::ConstPtr dynamic_map_msg, std::string map_frame_name, tf2_ros::Buffer& tfBuffer);
        void try_to_find_path();
        bool old_path_still_work(int start_x, int start_y);
        dstarlite(std::string map_topic);
        int from_real_x_to_map_x(double x){return (x - initial_x) / resolution;}
        int from_real_y_to_map_y(double y){return (y - initial_y) / resolution;}
        class Node{
            public:
                double dis_to_goal, manhattan_dis_to_start, rhs, f_for_astar, f_for_dstar;
                bool isObstacle, isStaticObstacle, reversed;
                int x, y, astar_list_status, dstar_list_status;
                int last_out_list_round, used_for_find_path;
                Nodeptr succ, father, son[2];
                Node(int x, int y){
                    dis_to_goal = INF + 1;
                    manhattan_dis_to_start = INF;//manhattan distance to oringin
                    rhs = INF;
                    f_for_astar = INF;
                    this->isObstacle = false;
                    this->x = x;
                    this->y = y;
                    astar_list_status = dstar_list_status = NEW;
                    last_out_list_round = 0;
                    succ = father = son[0] = son[1] = nullptr;
                }
                Node(){}
                void clear(){
                    dis_to_goal = INF + 1;
                    manhattan_dis_to_start = INF;
                    rhs = INF;
                    f_for_astar = INF;//f=dis+g
                    f_for_dstar = INF;//f=rhs+g
                    isObstacle = isStaticObstacle;
                    astar_list_status= dstar_list_status = NEW;
                    succ = father = son[0] = son[1] = nullptr;
                    reversed = false;
                }
                // class Compare_in_Astar{
                //     public:
                //         bool operator()(Nodeptr a, Nodeptr b){
                //             if(a->f_for_astar == b->f_for_astar){
                //                 if(a->dis_to_goal == b->dis_to_goal){
                //                     if(a->manhattan_dis_to_start == b->manhattan_dis_to_start){
                //                         if(a->x == b->x)return a->y < b->y;
                //                         return a->x < b->x;
                //                     }
                //                     return a->manhattan_dis_to_start < b->manhattan_dis_to_start;
                //                 }
                //                 return a->dis_to_goal < b->dis_to_goal;
                //             }
                //             return a->f_for_astar < b->f_for_astar;
                //         }
                // };
                class Compare_in_Dstar{
                    public:
                        bool operator()(Nodeptr a, Nodeptr b){
                            if(std::min(a->dis_to_goal, a->rhs) + a->manhattan_dis_to_start == std::min(b->dis_to_goal, b->rhs) + b->manhattan_dis_to_start){
                                if(std::min(a->dis_to_goal, a->rhs) == std::min(b->dis_to_goal, b->rhs)){
                                    if(a -> x == b -> x){
                                        return a -> y < b -> y;
                                    }
                                    return a -> x < b -> x;
                                }
                                return std::min(a->dis_to_goal, a->rhs) < std::min(b->dis_to_goal, b->rhs);
                            }
                            return std::min(a->dis_to_goal, a->rhs) + a->manhattan_dis_to_start < std::min(b->dis_to_goal, b->rhs) + b->manhattan_dis_to_start;
                        }
                };
        };
        class LinkCutTree{
            public:
                void reverse(Nodeptr now){now->reversed ^= 1;}
                bool get_son(Nodeptr x){return x->father->son[1] == x;}
                bool is_root(Nodeptr x){return x->father == nullptr || (x->father->son[0] != x && x->father->son[1] != x);}
                void rotate(Nodeptr x){
                    Nodeptr f = x->father, ff = f->father;
                    bool son = get_son(x);
                    if(!is_root(f)) ff->son[get_son(f)] = x; x -> father = ff;
                    f -> son[son] = x -> son[son^1]; if(x->son[son^1] != nullptr) x->son[son^1]->father = f;
                    x -> son[son^1] = f; f -> father = x;
                }
                void pushdown(Nodeptr now){
                    if(now->reversed){
                        now->reversed = false;
                        if(now->son[0] != nullptr) now->son[0]->reversed ^= 1;
                        if(now->son[1] != nullptr) now->son[1]->reversed ^= 1;
                        std::swap(now->son[0], now->son[1]);
                    }
                    return;
                }
                void splay(Nodeptr now){
                    std::stack<Nodeptr> s;
                    Nodeptr temp = now;
                    while(!is_root(temp)){
                        s.push(temp);
                        temp = temp->father;
                    }
                    s.push(temp);
                    while(!s.empty()){
                        pushdown(s.top());
                        s.pop();
                    }
                    while(!is_root(now)){
                        if(!is_root(now -> father)){
                            if(get_son(now->father) == get_son(now)) rotate(now->father);
                            else rotate(now);
                        }
                        rotate(now);
                    }
                }
                void access(Nodeptr now){
                    Nodeptr last = nullptr;
                    for(Nodeptr x = now; x != nullptr; x = x->father){
                        splay(x);
                        x->son[1] = last;
                        last = x;
                    }
                }
                void makeroot(Nodeptr now){
                    access(now);
                    splay(now);
                    reverse(now);
                }
                Nodeptr find_root(Nodeptr now){
                    access(now);
                    splay(now);
                    while(now->son[0] != nullptr) now = now->son[0];
                    return now;
                }
                void link(Nodeptr x, Nodeptr y){
                    // ROS_INFO("Link %d %d %d %d", x->x, x->y, y->x, y->y);
                    if(find_root(x) == find_root(y)){
                        // ROS_ERROR("Link Error");
                        // exit(1);
                        return ;
                    }
                    // ROS_INFO("Link %d %d %d %d", x->x, x->y, y->x, y->y);
                    makeroot(x);
                    makeroot(y);
                    x->father = y;
                }
                void del(Nodeptr x, Nodeptr y){
                    if(x == nullptr || y == nullptr)return;
                    makeroot(x);
                    access(y);
                    splay(y);
                    if(y->son[0] == x && x->son[1] == nullptr && x->son[0] == nullptr){ 
                        y->son[0] = nullptr;
                        x->father = nullptr;
                    }
                }
        };
        LinkCutTree* lct;
        Nodeptr final_goal_node, start_node, origin_start_node;
        int max_x, max_y, round, for_find_path_round;
        Nodeptr** map;
        // std::set<Nodeptr, Node::Compare_in_Astar> astar_list;
        std::set<Nodeptr, Node::Compare_in_Dstar> dstar_list;
        std::queue<Nodeptr> changed_obstacle_nodes;
        int dx[8] = {1, 0, -1, 0, 1, -1, -1, 1};
        int dy[8] = {0, 1, 0, -1, 1, -1, 1, -1};
        ros::Rate* rate_for_staight_line, *rate_for_diagonal_line;
        double velocity = 0.8, resolution, initial_x, initial_y, d_manhattan_dis_to_start;
};
dstarlite::dstarlite(std::string map_topic){
    ros::NodeHandle nh;
    nav_msgs::OccupancyGrid::ConstPtr msg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(map_topic, nh);
    ROS_INFO("get_map_started");
    if (msg != nullptr){
        round = 0;
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
        lct = new LinkCutTree();
        //can be further developed by changing the refreshing rate
    }
    else{
        ROS_ERROR("Failed to get map");
        return;
    }
    ROS_INFO("get_map_finished");
}
void dstarlite::when_receive_new_goal(geometry_msgs::PointStamped::ConstPtr goal_pose_msg, double real_start_x, double real_start_y){
    int start_x, start_y, goal_x, goal_y;
    goal_x = from_real_x_to_map_x(goal_pose_msg->point.x);
    goal_y = from_real_y_to_map_y(goal_pose_msg->point.y);
    start_x = from_real_x_to_map_x(real_start_x);
    start_y = from_real_y_to_map_y(real_start_y);
    final_goal_node = map[goal_x][goal_y];
    origin_start_node = start_node = map[start_x][start_y];
    d_manhattan_dis_to_start = 0;
    reset();
    dstar_list.clear();
    for(int i = 0; i < max_x; i++)for(int j = 0; j < max_y; j++)map[i][j]->manhattan_dis_to_start = abs(i - start_x) + abs(j - start_y);
    // map[goal_x][goal_y]->dis_to_goal = 0;
    map[goal_x][goal_y]->rhs = 0;
    dstar_list.insert(map[goal_x][goal_y]);
    map[goal_x][goal_y]->dstar_list_status = IN_LIST;
    // dstar_update(map[goal_x][goal_y]);
    //need code to deal with goal and start
    // dstar_main()
    // astar_main(start_x, start_y, goal_x, goal_y);
    //after that, we will make most nodes into the list and quit status new.
}
void dstarlite::when_receive_new_dynamic_map(nav_msgs::OccupancyGrid::ConstPtr dynamic_map_msg, std::string map_frame_name, tf2_ros::Buffer& tfBuffer){
    int x_size_for_dynamic_map = dynamic_map_msg->info.width;
    int y_size_for_dynamic_map = dynamic_map_msg->info.height;
    double resolution_for_dynamic_map = dynamic_map_msg->info.resolution;
    double initial_x_for_dynamic_map = dynamic_map_msg->info.origin.position.x;
    double initial_y_for_dynamic_map = dynamic_map_msg->info.origin.position.y;
    std::string dynamic_frame_name = dynamic_map_msg->header.frame_id;
    geometry_msgs::TransformStamped transformStamped;
    // std::cout << x_size_for_dynamic_map <<' '<< y_size_for_dynamic_map<<'\n';
    try {
        transformStamped = tfBuffer.lookupTransform(map_frame_name, dynamic_frame_name, ros::Time(0));
    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
    ROS_INFO("Erase obstacle nodes");
    while(!changed_obstacle_nodes.empty()){
        Nodeptr cur = changed_obstacle_nodes.front();
        changed_obstacle_nodes.pop();
        if(cur->dstar_list_status==IN_LIST){
            dstar_list.erase(cur);
            cur->dstar_list_status = OUT_LIST;
        }
        cur->isObstacle = 0;
        dstar_update_node(cur);
        // dstar_update(nullptr);
    }
    ROS_INFO("Deal with dynamic map");
    for(int i = 0; i < x_size_for_dynamic_map; i++){
        for(int j = 0; j < y_size_for_dynamic_map; j++){
            int index = i + j * x_size_for_dynamic_map;
            // if (dynamic_map_msg->data[index] == 100){
            double x_in_dynamic_map = i * resolution_for_dynamic_map + initial_x_for_dynamic_map;
            double y_in_dynamic_map = j * resolution_for_dynamic_map + initial_y_for_dynamic_map;
            tf2::Vector3 dynamic_map_vector = tf2::Vector3(x_in_dynamic_map, y_in_dynamic_map, 0);
            tf2::Transform tf_transform;
            tf2::fromMsg(transformStamped.transform, tf_transform);
            tf2::Vector3 map_vector = tf_transform * dynamic_map_vector;
            // ROS_INFO("dstar node: %lf %lf %d %d", map_vector.x(), map_vector.y(), x, y);
            int x = from_real_x_to_map_x(map_vector.x());
            int y = from_real_y_to_map_y(map_vector.y());
            // ROS_INFO("dstar node: %lf %lf %d %d", map_vector.x(), map_vector.y(), x, y);
            if(x < 0 || x >= max_x || y < 0 || y >= max_y)continue;
            bool is_obstacle = dynamic_map_msg->data[index] == 100;
            if(x == final_goal_node->x && y == final_goal_node->y){
                final_goal_node -> isObstacle = is_obstacle;
                continue;
            }
            if(map[x][y]->isObstacle == 0 && is_obstacle){
                if(map[x][y]->dstar_list_status == IN_LIST){
                    dstar_list.erase(map[x][y]);
                    map[x][y]->dstar_list_status = OUT_LIST;
                }
                map[x][y]->isObstacle = true;
                map[x][y]->rhs = INF;
                map[x][y]->dis_to_goal = INF + 1;
                map[x][y]->f_for_dstar = INF;
                map[x][y]->succ = nullptr;
                for(int k = 0; k < 8; k++){
                    int nx = x + dx[k];
                    int ny = y + dy[k];
                    if(nx < 0 || nx >= max_x || ny < 0 || ny >= max_y) continue;
                    if(map[nx][ny]->isObstacle)continue;
                    lct->del(map[nx][ny], map[x][y]);
                    dstar_update_node(map[nx][ny]);
                }
                changed_obstacle_nodes.push(map[x][y]);
            }
        }
    }
}
void dstarlite::reset(){for(int i = 0; i < max_x; i++)for(int j = 0; j < max_y; j++)map[i][j]->clear();}
// void dstarlite::astar_update_node(Nodeptr cur, double new_dis_to_goal, Nodeptr succ){
//     if(new_dis_to_goal < cur->dis_to_goal){
//         if(cur->astar_list_status == IN_LIST){
//             astar_list.erase(cur);
//             cur->astar_list_status = OUT_LIST;
//         }
//         cur->dis_to_goal = cur->rhs = new_dis_to_goal;
//         cur->f_for_astar = cur->f_for_dstar= cur->dis_to_goal + cur->manhattan_dis_to_start;
//         cur->succ = succ;
//         cur->astar_list_status = IN_LIST;
//         astar_list.insert(cur);
//     }
// }
// void dstarlite::astar_main(int start_x, int start_y, int goal_x, int goal_y){
//     reset();
//     astar_list.clear();
//     dstar_list.clear();
//     for(int i = 0; i < max_x; i++)for(int j = 0; j < max_y; j++)map[i][j]->manhattan_dis_to_start = abs(i - start_x) + abs(j - start_y);
//     map[goal_x][goal_y]->dis_to_goal = 0;
//     map[goal_x][goal_y]->f_for_astar = map[goal_x][goal_y]->f_for_dstar = map[goal_x][goal_y]->manhattan_dis_to_start;
//     map[goal_x][goal_y]->rhs = 0;
//     astar_list.insert(map[goal_x][goal_y]);
//     map[goal_x][goal_y]->astar_list_status = IN_LIST;
//     astar_update(map[start_x][start_y]);
// }
// void dstarlite::astar_update(Nodeptr end_node){
//     round++;
//     while(!astar_list.empty()){
//         Nodeptr cur = *astar_list.begin();
//         astar_list.erase(astar_list.begin());
//         cur->astar_list_status = OUT_LIST;
//         // if(cur->last_out_list_round == round)continue;
//         cur->last_out_list_round = round;
//         if(cur == end_node)break;
//         for(int i=0;i<8;i++){
//             int nx = cur->x + dx[i];
//             int ny = cur->y + dy[i];
//             if(nx < 0 || nx >= max_x || ny < 0 || ny >= max_y)continue;
//             if(map[nx][ny]->isObstacle)continue;
//             if(i<4)astar_update_node(map[nx][ny], cur->dis_to_goal + 1, cur);
//             else astar_update_node(map[nx][ny], cur->dis_to_goal + 1.414, cur);
//             //can be further developed by changing the distance between two nodes
//         }
//     }
// }

//----------------------------------------------------------------------------------------------------------------
// void dstarlite::dstar_update_node(Nodeptr cur, double new_rhs, Nodeptr succ){ //new_ths: changing rhs from adjacent point
//     if(new_rhs < cur->rhs){
//         if(cur->dstar_list_status == IN_LIST){
//             dstar_list.erase(cur);
//             cur->dstar_list_status = OUT_LIST;
//         }
//         cur->rhs = new_rhs;
//         cur->f_for_dstar= cur->rhs + cur->manhattan_dis_to_start;
//         cur->succ = succ;
//         cur->dstar_list_status = IN_LIST;
//         dstar_list.insert(cur);
//     }
// }
// void dstarlite::dstar_update_node(Nodeptr cur, Nodeptr succ){//
//     // if(succ == cur->succ){
//         if(cur->dstar_list_status == IN_LIST){
//             dstar_list.erase(cur);
//             cur->dstar_list_status = OUT_LIST;
//         }
//         cur->rhs = INF;
//         cur->succ = nullptr;
//         for(int i = 0; i < 8; i++){
//             int nx = cur->x + dx[i];
//             int ny = cur->y + dy[i];
//             if(nx < 0 || nx >= max_x || ny < 0 || ny >= max_y)continue;
//             if(map[nx][ny]->isObstacle)continue;
//             double new_rhs = i<4?map[nx][ny]->rhs + 1 : map[nx][ny]->rhs + 1.414;
//             if(new_rhs < cur->rhs){
//                 cur->rhs = new_rhs;
//                 cur->succ = map[nx][ny];
//             }
//         }
//         cur->f_for_dstar = cur->rhs + cur->manhattan_dis_to_start;
//         cur->dstar_list_status = IN_LIST;
//         dstar_list.insert(cur);
//     // }
// }
void dstarlite::dstar_update_node(Nodeptr cur){
    if(cur->isObstacle || cur == final_goal_node)return;
    double value = 1;
    if(cur->dstar_list_status == IN_LIST){
        dstar_list.erase(cur);
        cur->dstar_list_status = OUT_LIST;
    }
    cur->manhattan_dis_to_start = abs(cur->x - start_node->x) + abs(cur->y - start_node->y) + d_manhattan_dis_to_start;
    lct->del(cur, cur->succ);
    cur->rhs = INF;
    cur->succ = nullptr;
    for(int i = 0; i < 8; i++){
        int nx = cur->x + dx[i];
        int ny = cur->y + dy[i];
        if(nx < 0 || nx >= max_x || ny < 0 || ny >= max_y)continue;
        if(map[nx][ny]->isObstacle)continue;
        double new_rhs = i<4?map[nx][ny]->dis_to_goal + value : map[nx][ny]->dis_to_goal + 1.414 * value;
        if(new_rhs < cur->rhs){
            cur->rhs = new_rhs;
            cur->succ = map[nx][ny];
        }
    }
    if(cur->rhs != cur->dis_to_goal){
        for(int i = 0; i < 8; i++){
            int nx = cur->x + dx[i];
            int ny = cur->y + dy[i];
            if(nx < 0 || nx >= max_x || ny < 0 || ny >= max_y)continue;
            if(map[nx][ny]->isObstacle)continue;
            lct->del(cur, map[nx][ny]);
        }
    }
    else{
        for(int i = 0; i < 8; i++){
            int nx = cur->x + dx[i];
            int ny = cur->y + dy[i];
            if(nx < 0 || nx >= max_x || ny < 0 || ny >= max_y)continue;
            if(map[nx][ny]->isObstacle)continue;
            if(map[nx][ny]->succ == cur && map[nx][ny]->dis_to_goal == map[nx][ny]->rhs)lct->link(map[nx][ny], cur);
        }
    }
    cur->f_for_dstar = cur->rhs + cur->manhattan_dis_to_start;
    cur->dstar_list_status = IN_LIST;
    dstar_list.insert(cur);
    if(cur -> succ != nullptr && cur -> dis_to_goal == cur -> rhs && cur -> succ -> dis_to_goal == cur -> succ -> rhs)lct->link(cur, cur->succ);
}

void dstarlite::dstar_main(nav_msgs::OccupancyGrid::ConstPtr dynamic_map_msg, double real_start_x, double real_start_y, std::string map_frame_name, tf2_ros::Buffer& tfBuffer){
    int start_x = from_real_x_to_map_x(real_start_x);
    int start_y = from_real_y_to_map_y(real_start_y);
    start_node = map[start_x][start_y];
    d_manhattan_dis_to_start = abs(start_x - origin_start_node->x) + abs(start_y - origin_start_node->y);
    ROS_INFO("start_deal_with_dynamic_map");
    when_receive_new_dynamic_map(dynamic_map_msg, map_frame_name, tfBuffer);
    ROS_INFO("finish");
    if(start_node->isObstacle || final_goal_node->isObstacle){
        ROS_ERROR("Start or final node is obstacle");
        return;
    }
    if(old_path_still_work(start_x, start_y))return;
    ROS_INFO("start_pos: %d %d", start_x, start_y);
    dstar_update(map[start_x][start_y]);
    ROS_INFO("dstar_update_finished");
}
void dstarlite::dstar_update(Nodeptr end_node){
    ROS_INFO("dstar_list size: %d", dstar_list.size());
    int count = 0;
    round++;
    ROS_INFO("origin_start_node_status: %lf %lf %d %d", end_node->dis_to_goal, end_node->rhs, end_node->dstar_list_status, end_node->isObstacle);
    while(!dstar_list.empty()){
        count++;
        Nodeptr cur = *dstar_list.begin();
        dstar_list.erase(cur);
        // ROS_INFO("Current_node %d %d %lf %lf %d %d", cur->x, cur->y, cur->dis_to_goal, cur->rhs, final_goal_node->x, final_goal_node->y);
        // ros::Duration(0.1).sleep();
        cur->last_out_list_round = round;
        cur->dstar_list_status = OUT_LIST;
        if(cur->dis_to_goal < cur -> rhs){
            int x = cur->x, y = cur->y;
            cur -> dis_to_goal = INF + 1;
            cur -> dstar_list_status = IN_LIST;
            cur->manhattan_dis_to_start = abs(cur->x - start_node->x) + abs(cur->y - start_node->y) + d_manhattan_dis_to_start;
            dstar_list.insert(cur);
            for(int i = 0; i < 8; i++){
                int nx = x + dx[i];
                int ny = y + dy[i];
                if(nx < 0 || nx >= max_x || ny < 0 || ny >= max_y)continue;
                dstar_update_node(map[nx][ny]);
            }
        }
        else if(cur -> dis_to_goal > cur -> rhs){
            int x = cur->x, y = cur->y;
            cur -> dis_to_goal = cur -> rhs;
            for(int i = 0; i < 8; i++){
                int nx = x + dx[i];
                int ny = y + dy[i];
                if(nx < 0 || nx >= max_x || ny < 0 || ny >= max_y)continue;
                dstar_update_node(map[nx][ny]);
            }
        }
        if(cur -> dis_to_goal == cur -> rhs && cur -> succ != nullptr && cur -> succ -> dis_to_goal == cur -> succ -> rhs)lct->link(cur, cur->succ);
        if(lct->find_root(final_goal_node) == lct->find_root(end_node))break;
        // if(cur == end_node && cur->dstar_list_status == OUT_LIST)break;
        // if(end_node->dis_to_goal == end_node->rhs)break;
        // if(count%10000 == 0){
        //     try_to_find_path();
        //     publish_all_map_status(*pub_map);
        //     ROS_INFO("start_node_status: %lf %lf %d %d %d %d %p", end_node->dis_to_goal, end_node->rhs, end_node->dstar_list_status, end_node->isObstacle, round, end_node->last_out_list_round, end_node->succ);
        //     ROS_INFO("end_node_status: %lf %lf %d %d %d %d %p", final_goal_node->dis_to_goal, final_goal_node->rhs, final_goal_node->dstar_list_status, final_goal_node->isObstacle, round, final_goal_node->last_out_list_round, final_goal_node->succ);
        //     ros::Duration(1).sleep();
        // }
    }
    ROS_INFO("dstar_list size end with: %d", dstar_list.size());
    // if(!dstar_list.empty()){
    //     ROS_ERROR("dstar_list is not empty");
    //     exit(1);
    // }
}
void dstarlite::publish(ros::Publisher& pub, std::string map_frame_name, ros::Publisher& cmd_vel_pub, std::string robot_frame_name, tf2_ros::Buffer& tfBuffer){
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = map_frame_name; 
    ROS_INFO("Publish started");
    Nodeptr cur = start_node;
    geometry_msgs::Twist cmd_vel;
    geometry_msgs::TransformStamped transformStamped;
    try {
        transformStamped = tfBuffer.lookupTransform(robot_frame_name, map_frame_name, ros::Time(0));
    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
    ROS_INFO("Find root");
    if(cur == nullptr || final_goal_node == nullptr) ROS_ERROR("?????");
    if(lct->find_root(cur) == lct->find_root(final_goal_node)){
        ROS_INFO("cur->succ:%p cur:%p root(cur)%p goal%p root(goal)%p", cur->succ, cur, lct->find_root(cur), final_goal_node, lct->find_root(final_goal_node));
        double dx = cur->succ->x - cur->x;
        double dy = cur->succ->y - cur->y;
        double dis = sqrt(dx * dx + dy * dy);
        cmd_vel.linear.x = dx / dis * velocity;
        cmd_vel.linear.y = dy / dis * velocity;
        tf2::Vector3 old_cmd_vel = tf2::Vector3(cmd_vel.linear.x, cmd_vel.linear.y, 0);
        tf2::Transform tf_transform;
        transformStamped.transform.translation.x = 0;
        transformStamped.transform.translation.y = 0;
        transformStamped.transform.translation.z = 0;
        tf2::fromMsg(transformStamped.transform, tf_transform);
        tf2::Vector3 new_cmd_vel = tf_transform * old_cmd_vel;
        cmd_vel.linear.x = new_cmd_vel.x();
        cmd_vel.linear.y = -new_cmd_vel.y();
        cmd_vel_pub.publish(cmd_vel);
    }
    else{
        cmd_vel.linear.x = 0;
        cmd_vel.linear.y = 0;
        cmd_vel_pub.publish(cmd_vel);
        return;
    }
    ROS_INFO("Find path");
    int count = 0;
    while(cur != nullptr && cur != final_goal_node){
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = cur->x * resolution + initial_x;
        pose.pose.position.y = cur->y * resolution + initial_y;
        pose.pose.position.z = 0;
        path.poses.push_back(pose);
        // count++;
        // if(count < 100)ROS_INFO("path: %lf %lf %d %d %lf %lf %d %d %d %d", pose.pose.position.x, pose.pose.position.y, cur->x, cur->y, cur->dis_to_goal, cur->rhs, cur->last_out_list_round, round, cur->dstar_list_status, (int)cur->isObstacle) ;
        cur = cur->succ;
    }
    pub.publish(path);
    ROS_INFO("Publish finished");
}
void dstarlite::publish_all_map_status(ros::Publisher& pub){
    nav_msgs::OccupancyGrid all_map_status;
    all_map_status.header.stamp = ros::Time::now();
    all_map_status.header.frame_id = "map";
    all_map_status.info.width = max_x;
    all_map_status.info.height = max_y;
    all_map_status.info.resolution = resolution;
    all_map_status.info.origin.position.x = initial_x;
    all_map_status.info.origin.position.y = initial_y;
    all_map_status.info.origin.position.z = 0;
    all_map_status.info.origin.orientation.x = 0;
    all_map_status.info.origin.orientation.y = 0;
    all_map_status.info.origin.orientation.z = 0;
    all_map_status.info.origin.orientation.w = 1;
    all_map_status.data.resize(max_x * max_y);
    for(int i = 0; i < max_x; i++){
        for(int j = 0; j < max_y; j++){
            int index = i + j * max_x;
            // if(map[i][j]->isObstacle)all_map_status.data[index] = 100;
            if(map[i][j]->last_out_list_round == round && !map[i][j]->isObstacle)all_map_status.data[index] = 100;
            else all_map_status.data[index] = 0;
        }
    }
    pub.publish(all_map_status);

}
bool dstarlite::old_path_still_work(int start_x, int start_y){
    Nodeptr cur = map[start_x][start_y];
    return lct->find_root(cur) == lct->find_root(final_goal_node);
}
bool get_dynamic_map_info = false;
nav_msgs::OccupancyGrid::ConstPtr dynamic_map_msg;
void record_dynamic_map_info(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    dynamic_map_msg = msg;    
    get_dynamic_map_info = true;
    // ROS_INFO("get dynamic map info");
}
void dstarlite::try_to_find_path(){
    Nodeptr cur = start_node;
    // ROS_INFO("%lf %lf %d", cur->x, cur->y, cur);
    int count = 0;
    for_find_path_round++;
    ROS_INFO("try_to_find_path");
    while(cur!=nullptr && cur!=final_goal_node){
        if(cur->isObstacle || cur -> dis_to_goal != cur -> rhs){
            ROS_INFO("Obstacle");
            return;
        }
        if(cur->used_for_find_path == round){
            ROS_INFO("Loop");
            return;
        }
        if(lct->find_root(cur) != lct->find_root(cur->succ)){
            ROS_INFO("Not connected");
            // ros::Duration(1).sleep();   
        }
        cur->used_for_find_path =for_find_path_round;
        if(count < 50)ROS_INFO("%d %d %d %lf %lf", cur->x, cur->y, cur->last_out_list_round, cur->dis_to_goal, cur->rhs);
        cur = cur->succ;
        count++;
    }
    ROS_INFO("Find path");

    return;
}
geometry_msgs::PointStamped::ConstPtr goal_pose_msg;
bool get_goal_info = false;
void record_goal_info(const geometry_msgs::PointStamped::ConstPtr& msg){
    goal_pose_msg = msg;
    get_goal_info = true;
    ROS_INFO("get goal info");
}
int main(int argc, char **argv){
    std::string node_name = "dstarlite";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    std::string static_map_topic_name;
    if (!nh.getParam(node_name+"/"+"map_topic_name", static_map_topic_name)) ROS_ERROR("Failed to get param 'map_topic_name'");
    ROS_INFO("static_map_topic_name: %s", static_map_topic_name.c_str());
    
    std::string map_frame_name;
    if (!nh.getParam(node_name+"/"+"map_frame_name", map_frame_name)) ROS_ERROR("Failed to get param 'map_frame_name'");
    ROS_INFO("map_frame_name: %s", map_frame_name.c_str());
    
    std::string robot_frame_name;
    if (!nh.getParam(node_name+"/"+"robot_frame_name", robot_frame_name)) ROS_ERROR("Failed to get param 'robot_frame_name'");
    ROS_INFO("robot_frame_name: %s", robot_frame_name.c_str());
    
    std::string dynamic_map_topic_name;
    if (!nh.getParam(node_name+"/"+"dynamic_map_topic_name", dynamic_map_topic_name)) ROS_ERROR("Failed to get param 'dynamic_map_topic_name'");
    ros::Subscriber dynamic_map_sub = nh.subscribe(dynamic_map_topic_name, 1, record_dynamic_map_info);
    ROS_INFO("dynamic_map_topic_name: %s", dynamic_map_topic_name.c_str());
    
    std::string goal_topic_name;
    if (!nh.getParam(node_name+"/"+"goal_topic_name", goal_topic_name)) ROS_ERROR("Failed to get param 'goal_topic_name'");
    ros::Subscriber goal_sub = nh.subscribe(goal_topic_name, 1, record_goal_info);
    ROS_INFO("goal_topic_name: %s", goal_topic_name.c_str());
    
    ros::Publisher pub = nh.advertise<nav_msgs::Path>("dstar_path", 1);
    ros::Publisher pub2 = nh.advertise<nav_msgs::OccupancyGrid>("all_map_status", 1);
    ros::Publisher pub3 = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    pub_map = &pub2;

    ROS_INFO("Initialization started");
    dstarlite dstar(static_map_topic_name);
    ROS_INFO("Initialization finished");
    bool have_first_goal = false;
    ros::Rate rate(50);
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped;
    while(true){
        // ROS_INFO("loop");
        if(get_goal_info && get_dynamic_map_info){
            try {
                transformStamped = tfBuffer.lookupTransform(map_frame_name, robot_frame_name, ros::Time(0));
            } catch (tf2::TransformException &ex) {
                ROS_WARN("%s", ex.what());
                ros::Duration(1.0).sleep();
            }
            ROS_INFO("get goal and dynamic map info");
            dstar.when_receive_new_goal(goal_pose_msg, transformStamped.transform.translation.x, transformStamped.transform.translation.y);
            // dstar.dstar_main(dynamic_map_msg, transformStamped.transform.translation.x, transformStamped.transform.translation.y, map_frame_name, tfBuffer);
            // dstar.publish(transformStamped.transform.translation.x, transformStamped.transform.translation.y, pub, map_frame_name);
            get_goal_info = false;
            have_first_goal = true;
        }
        if(have_first_goal && get_dynamic_map_info){
            try {
                transformStamped = tfBuffer.lookupTransform(map_frame_name, robot_frame_name, ros::Time(0));
            } catch (tf2::TransformException &ex) {
                ROS_WARN("%s", ex.what());
                ROS_INFO("ERROR IN MAP TO ROBOT");
                ros::Duration(1.0).sleep();
            }
            double real_goal_x = dstar.final_goal_node->x * dstar.resolution + dstar.initial_x;
            double real_goal_y = dstar.final_goal_node->y * dstar.resolution + dstar.initial_y;
            if(abs(real_goal_x - transformStamped.transform.translation.x) < 0.1 && abs(real_goal_y - transformStamped.transform.translation.y) < 0.1){
                ROS_INFO("Goal reached");
                have_first_goal = false;
                geometry_msgs::Twist cmd_vel;
                cmd_vel.linear.x = 0;
                cmd_vel.linear.y = 0;
                pub3.publish(cmd_vel);
                continue;
            }
            ROS_INFO("get dynamic map info");
            dstar.dstar_main(dynamic_map_msg, transformStamped.transform.translation.x, transformStamped.transform.translation.y, map_frame_name, tfBuffer);
            dstar.publish(pub, map_frame_name, pub3, robot_frame_name, tfBuffer);
            get_dynamic_map_info = false;
        }
        // dstar.publish_all_map_status(pub2);
        ROS_INFO("__________________________");
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}