#include "ros/ros.h"
#include "decision/Armor.h"
#include "decision/Armors.h"
#include "decision/Target.h"
#include "decision/TargetCommand.h"
#include "decision/GameStats.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <actionlib/client/simple_action_client.h>
#include "bot_sim/NavActionAction.h" // bot_sim
#include <string>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <chrono>

ros::Publisher pub;

std::vector<decision::Armor> armors;
decision::Target target;
decision::GameStats game_stats;
geometry_msgs::Point position;
int last_hp;

typedef actionlib::SimpleActionClient<bot_sim::NavActionAction> NavClient;

enum class DecisionState {
    PATROLLING,
    TRACKING,
    HEALING,
    RETURNING
};

struct Events {
    bool captured_supply;
    bool captured_central_buff;
    bool opponent_captured_central_buff;
} game_events;

void armorCallback(const decision::Armors::ConstPtr& armors_msg)
{
    armors = armors_msg->armors;
}

void targetCallback(const decision::Target::ConstPtr& target_msg)
{
    target = *target_msg;
}

void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg) {
    position.x = pose_msg->pose.pose.position.x;
    position.y = pose_msg->pose.pose.position.y;

    tf2::Quaternion q;
    tf2::fromMsg(pose_msg->pose.pose.orientation, q);

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    position.z = yaw;
}

// void basicAttack(const decision::Armors::ConstPtr& armors_msg)
// { //this function will choose the enemy that have the lowest HP and attack it's closest armor 
// // Assume we have a function GetHP(int id) that returns the HP of the enemy with the given id
//     detector::TargetCommand cmd_msg;
//     double min_sq_distance = DBL_MAX; 
//     int min_hp = DBL_MAX;

//     // choose the nearest armor with the lowest HP
//     if (armors_msg->armors.size() > 0) {
//         for (const auto & armor : armors_msg->armors) {
//             double sq_distance = armor.pose.position.x * armor.pose.position.x + armor.pose.position.y * armor.pose.position.y;
//             int HP = GetHP(armor.id);
//             if (HP <min_hp){
//                 min_hp = HP;
//                 cmd_msg.id = armor.id; 
//             } else if (HP == min_hp && sq_distance < min_sq_distance){
//                 cmd_msg.id = armor.id;
//             } 
//         }
//     }else{
//          cmd_msg.id = 0;
//     }
//     // decide whether to fire
//     // assume we have a function GetRemainingBulletNumber() that returns the remaining bullet number
//     int bullet_number = GetRemainingBulletNumber();
//     // close_enough, HP_low_enough
//     if cmd_msg.id == 0 {
//         cmd_msg.permitFiring = false;
//     }else{
//         if (!RunningOutOfBullet()){
//             // fire without hesitation
//             cmd_msg.permitFiring = true;
//         } else if (RunningOutOfBullet() && bullet_number > 0){
//             // fire only if the enemy got low HP and close enough 
//             if (min_hp < 100 && sq_distance < 9){
//                 cmd_msg.permitFiring = true;
//             } else {
//                 cmd_msg.permitFiring = false;
//             }
//         }else {
//             // no bullet left
//             cmd_msg.permitFiring = false;
//         }
//     }
//     pub.publish(cmd_msg);
// }

bool runningOutOfBullets(){
    return false;
    // time in minutes
    int bullet_number = 750; // bullets
    double time = game_stats.stage_remain_time / 60.0;
    if (bullet_number < 0.7 * (750 - 150 * (time + 1.0))){
        return true; 
    } else {
        return false; 
    }
}

int getNearestWaypoint(const std::vector<std::vector<double>> & patrol_path, const geometry_msgs::Point & position)
{
    double min_sq_distance = DBL_MAX;
    int nearest_waypoint = 0;
    for (int i = 0; i < patrol_path.size(); ++i) {
        double x_diff = patrol_path[i][0] - position.x;
        double y_diff = patrol_path[i][1] - position.y;
        double sq_distance = x_diff * x_diff + y_diff * y_diff;
        if (sq_distance < min_sq_distance) {
            min_sq_distance = sq_distance;
            nearest_waypoint = i;
        }
    }
    return nearest_waypoint;
}

void gamestatsCallback(const decision::GameStats::ConstPtr& gamestats_msg)
{
    // std::cout << "Game time: " << gamestats_msg->game_type << std::endl;
    // std::cout << "Game progress: " << gamestats_msg->game_progress << std::endl;
    // ROS_INFO("game_type: %d", gamestats_msg->game_type);
    // ROS_INFO("remain_hp: %d", gamestats_msg->remain_hp);
    // ROS_INFO("max_hp: %d", gamestats_msg->max_hp);;
    game_stats = *gamestats_msg;
    game_events.captured_supply = (game_stats.event_data >> 2) & 1;
    game_events.captured_central_buff = (game_stats.event_data >> 30) & 1;
    game_events.opponent_captured_central_buff = (game_stats.event_data >> 31) & 1;
    // ROS_INFO("Captured supply: %d", game_events.captured_supply);
    // ROS_INFO("Captured central buff: %d", game_events.captured_central_buff);
    // ROS_INFO("Opponent captured central buff: %d", game_events.opponent_captured_central_buff);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "decision_node");

    ros::NodeHandle n;

    XmlRpc::XmlRpcValue patrol_path_list;
    if (!n.getParam("patrol_path", patrol_path_list)) {
        ROS_ERROR("Failed to get param 'patrol_path'");
    }

    std::vector<std::vector<double>> patrol_path;
    for (int32_t i = 0; i < patrol_path_list.size(); ++i) {
        std::vector<double> point;
        for (int32_t j = 0; j < patrol_path_list[i].size(); ++j) {
            point.push_back(static_cast<double>(patrol_path_list[i][j]));
        }
        patrol_path.push_back(point);
    }

    XmlRpc::XmlRpcValue control_location_list;
    std::vector<double> control_location;
    if (!n.getParam("control_location", control_location_list)) {
        ROS_ERROR("Failed to get param 'control_location'");
    }
    for(int i = 0; i < control_location_list.size(); i++)
    {
        std::cout << control_location_list[i] << std::endl;
        control_location.push_back(static_cast<double>(control_location_list[i]));
    }

    int hp_before_returning;
    if (!n.getParam("hp_before_returning", hp_before_returning)) {
        ROS_ERROR("Failed to get param 'hp_before_returning'");
    }

    int mode;
    if (!n.getParam("mode", mode)) {
        ROS_ERROR("Failed to get param 'mode'");
    }

    std::cout << "Currently at mode: " << mode << std::endl;

    XmlRpc::XmlRpcValue base_location_list;
    std::vector<double> base_location;
    if (!n.getParam("base_location", base_location_list)) {
        ROS_ERROR("Failed to get param 'base_location'");
    }

    // XmlRpc::XmlRpcValue base_location_list;
    // std::vector<double> base_location;
    // if (!n.getParam("base_location_list", base_location_list)) {
    //     ROS_ERROR("Failed to get param 'supplies_location'");
    // }
    // for(int i = 0; i < base_location_list.size(); i++)
    // {
    //     std::cout << base_location_list[i] << std::endl;
    //     base_location.push_back(static_cast<double>(base_location[i]));
    // }

    XmlRpc::XmlRpcValue supplies_location_list;
    std::vector<double> supplies_location;
    if (!n.getParam("supplies_location", supplies_location_list)) {
        ROS_ERROR("Failed to get param 'supplies_location'");
    }
    for(int i = 0; i < supplies_location_list.size(); i++)
    {
        std::cout << supplies_location_list[i] << std::endl;
        supplies_location.push_back(static_cast<double>(supplies_location_list[i]));
    }

    pub = n.advertise<decision::TargetCommand>("/decision/target_cmd", 10);

    ros::Subscriber armors_sub = n.subscribe("/detector/armors", 10, armorCallback);

    ros::Subscriber gamestats_sub = n.subscribe("/decision/gamestats", 10, gamestatsCallback);

    ros::Subscriber pose_sub = n.subscribe("/amcl_pose", 10, poseCallback);

    NavClient nav_client("nav_ctrl", true);

    nav_client.waitForServer();

    DecisionState decision_state = DecisionState::PATROLLING;
    int current_waypoint = 0;
    bool waypoint_changed = true;
    bot_sim::NavActionGoal goal;
    // tf2_ros::Buffer tfBuffer;
    // tf2_ros::TransformListener tfListener(tfBuffer);
    // geometry_msgs::TransformStamped transformStamped;
    geometry_msgs::PoseWithCovarianceStamped pose_msg;
    std::chrono::steady_clock::time_point last_goal_time = std::chrono::steady_clock::now();

    // std::cout << 123 << std::endl;

    while (ros::ok()) {
        // try
        // {
        //     transformStamped = tfBuffer.lookupTransform("map", "virtual_frame", ros::Time(0));
        // }
        // catch (tf2::TransformException &ex)
        // {
        //     std::cout << "Error: " << ex.what() << std::endl;
        // }

        // std::cout << "Transform: " << transformStamped.transform.translation.x << " "
            // << transformStamped.transform.translation.y << std::endl;
        // position.x = transformStamped.transform.translation.x;
        // position.y = transformStamped.transform.translation.y;

        // std::cout << "Game progress: " << (int)game_stats.game_progress << std::endl;
        
        if (game_stats.game_progress != 4) {
            ros::spinOnce();
            continue;
        }

        // std::cout << "Entering loop" << std::endl;

        if (mode == 1 && decision_state == DecisionState::PATROLLING && waypoint_changed == true) {
            goal.goal_x = patrol_path[current_waypoint][0];
            goal.goal_y = patrol_path[current_waypoint][1];
            std::cout << "Sending goal: (" << goal.goal_x << ", " << goal.goal_y << ")" << std::endl;
            std::cout << "Current position: (" << position.x << ", " << position.y << ")" << std::endl;
            nav_client.sendGoal(goal);
            waypoint_changed = false;
        }

        if (mode == 2 && decision_state == DecisionState::PATROLLING) {
            goal.goal_x = control_location[0];
            goal.goal_y = control_location[1];
            std::cout << "Sending control point: (" << goal.goal_x << ", " << goal.goal_y << ")" << std::endl;
            std::cout << "Current position: (" << position.x << ", " << position.y << ")" << std::endl;
            nav_client.sendGoal(goal);
            waypoint_changed = false;
        }

        auto now = std::chrono::steady_clock::now();
        auto time_diff = std::chrono::duration_cast<std::chrono::seconds>(now - last_goal_time);

        if (mode == 1 && decision_state == DecisionState::PATROLLING && time_diff.count() >= 5) {
            goal.goal_x = patrol_path[current_waypoint][0];
            goal.goal_y = patrol_path[current_waypoint][1];
            std::cout << "Resending goal: (" << goal.goal_x << ", " << goal.goal_y << ")" << std::endl;
            std::cout << "Current position: (" << position.x << ", " << position.y << ")" << std::endl;
            nav_client.sendGoal(goal);
            // Update the last goal time
            last_goal_time = now;
        }

        if (mode == 2 && decision_state == DecisionState::PATROLLING && time_diff.count() >= 5) {
            goal.goal_x = control_location[0];
            goal.goal_y = control_location[1];
            std::cout << "Resending control point: (" << goal.goal_x << ", " << goal.goal_y << ")" << std::endl;
            std::cout << "Current position: (" << position.x << ", " << position.y << ")" << std::endl;
            nav_client.sendGoal(goal);
            waypoint_changed = false;
        }
        // std::cout << waypoint_changed << std::endl;

        // bool finished_before_timeout = nav_client.waitForResult(ros::Duration(30.0));

        // if (finished_before_timeout)
        // {
        //     actionlib::SimpleClientGoalState state = nav_client.getState();
        //     ROS_INFO("Action finished: %s",state.toString().c_str());
        // }
        // else
        //     ROS_INFO("Action did not finish before the time out.");

        decision::TargetCommand cmd_msg;
        double min_sq_distance = DBL_MAX;
        int proposed_target_id = 0;
        if (armors.size() > 0) {
            for (const auto & armor : armors) {
                double sq_distance = armor.pose.position.x * armor.pose.position.x + armor.pose.position.y * armor.pose.position.y;
                if (sq_distance < min_sq_distance) {
                    proposed_target_id = armor.id;
                }
            }
        }

        if (false /*gamestats.remain_hp < hp_before_returning*/) {
            // if there is enough supplies
            decision_state = DecisionState::HEALING;
        }
        else if (runningOutOfBullets()) {
            decision_state = DecisionState::RETURNING;
        }

        if (decision_state == DecisionState::HEALING && game_stats.remain_hp >= hp_before_returning) {
            decision_state = DecisionState::PATROLLING;
        }

        if (decision_state == DecisionState::RETURNING && game_stats.remain_hp >= hp_before_returning && !runningOutOfBullets()) {
            decision_state = DecisionState::PATROLLING;
        }

        if (decision_state == DecisionState::PATROLLING || decision_state == DecisionState::TRACKING || decision_state == DecisionState::RETURNING) {
            if (target.tracking) {
                cmd_msg.id = std::stoi(target.id);
                decision_state = DecisionState::TRACKING;
                // goal.goal_x = position.x * cos(position.z) - position.y * sin(position.z) + target.position.x;
                // goal.goal_y = position.y * sin(position.z) + position.y * cos(position.z) + target.position.y;
                // nav_client.sendGoal(goal);
                nav_client.cancelAllGoals();
                if (true /*gamestats.heat0 <= gamestats.heat0_limit - 20*/) {
                    cmd_msg.permitFiring = false; // to be modified to true
                }
                else {
                    cmd_msg.permitFiring = false;
                }
            }
            else if (proposed_target_id != 0) {
                decision_state = DecisionState::TRACKING;
                nav_client.cancelAllGoals();
                cmd_msg.id = proposed_target_id;
                cmd_msg.permitFiring = false;
            }
            else if (decision_state == DecisionState::TRACKING) {
                decision_state = DecisionState::PATROLLING;
                // current_waypoint = getNearestWaypoint(patrol_path, position);
                // waypoint_changed = true;
                cmd_msg.id = 0;
                cmd_msg.permitFiring = false;
            }
        }
        else {
            cmd_msg.id = 0;
            cmd_msg.permitFiring = false;
        }

        if (decision_state == DecisionState::RETURNING) {
            goal.goal_x = base_location[0];
            goal.goal_y = base_location[1];
            nav_client.sendGoal(goal);
        }
        else if (decision_state == DecisionState::HEALING) {
            goal.goal_x = supplies_location[0];
            goal.goal_y = supplies_location[1];
            nav_client.sendGoal(goal);
        }

        pub.publish(cmd_msg);

        if (mode == 1 && decision_state == DecisionState::PATROLLING && nav_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            waypoint_changed = true;
            current_waypoint = (current_waypoint + 1) % patrol_path.size();
        }

        ros::spinOnce();
    }

    return 0;
}
