#include "ros/ros.h"
#include "decision/Armor.h"
#include "decision/Armors.h"
#include "decision/Target.h"
#include "decision/TargetCommand.h"
#include "decision/GameStats.h"
#include <actionlib/client/simple_action_client.h>
#include "bot_sim/NavActionAction.h" // bot_sim
#include <string>

ros::Publisher pub;

std::vector<decision::Armor> armors;
decision::Target target;
decision::GameStats game_stats;

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

int getNearestWaypoint(const std::vector<std::vector<double>> & patrol_path)
{
    double min_sq_distance = DBL_MAX;
    int nearest_waypoint = 0;
    for (int i = 0; i < patrol_path.size(); ++i) {
        double sq_distance = patrol_path[i][0] * patrol_path[i][0] + patrol_path[i][1] * patrol_path[i][1];
        if (sq_distance < min_sq_distance) {
            min_sq_distance = sq_distance;
            nearest_waypoint = i;
        }
    }
    return nearest_waypoint;
}

void gamestatsCallback(const decision::GameStats::ConstPtr& gamestats_msg)
{
    ROS_INFO("Received game statistics:");
    ROS_INFO("game_type: %d", gamestats_msg->game_type);
    ROS_INFO("game_progress: %d", gamestats_msg->game_progress);
    ROS_INFO("stage_remain_time: %d", gamestats_msg->stage_remain_time);
    // ROS_INFO("red_1_hp: %d", gamestats_msg->red_1_hp);
    // ROS_INFO("red_2_hp: %d", gamestats_msg->red_2_hp);
    // ROS_INFO("red_3_hp: %d", gamestats_msg->red_3_hp);
    // ROS_INFO("red_4_hp: %d", gamestats_msg->red_4_hp);
    // ROS_INFO("red_5_hp: %d", gamestats_msg->red_5_hp);
    // ROS_INFO("red_7_hp: %d", gamestats_msg->red_7_hp);
    // ROS_INFO("blue_1_hp: %d", gamestats_msg->blue_1_hp);
    // ROS_INFO("blue_2_hp: %d", gamestats_msg->blue_2_hp);
    // ROS_INFO("blue_3_hp: %d", gamestats_msg->blue_3_hp);
    // ROS_INFO("blue_4_hp: %d", gamestats_msg->blue_4_hp);
    // ROS_INFO("blue_5_hp: %d", gamestats_msg->blue_5_hp);
    // ROS_INFO("blue_7_hp: %d", gamestats_msg->blue_7_hp);
    /*uint16 red_base_hp
uint16 blue_base_hp

uint16 remain_hp
uint16 max_hp
uint16 max_power
uint16 heat0_limit
uint16 heat0
float32 power
float32 buffer
float32 bullet_speed
uint8 shooter_speed_limit

uint32 event_data*/
    // ROS_INFO("red_base_hp: %d", gamestats_msg->red_base_hp);
    // ROS_INFO("blue_base_hp: %d", gamestats_msg->blue_base_hp);
    // ROS_INFO("remain_hp: %d", gamestats_msg->remain_hp);
    // ROS_INFO("max_hp: %d", gamestats_msg->max_hp);
    // ROS_INFO("max_power: %d", gamestats_msg->max_power);
    // ROS_INFO("heat0_limit: %d", gamestats_msg->heat0_limit);
    // ROS_INFO("heat0: %d", gamestats_msg->heat0);
    ROS_INFO("power: %f", gamestats_msg->power);
    // ROS_INFO("buffer: %f", gamestats_msg->buffer);
    // ROS_INFO("bullet_speed: %f", gamestats_msg->bullet_speed);
    // ROS_INFO("shooter_speed_limit: %d", gamestats_msg->shooter_speed_limit);
    game_stats = *gamestats_msg;
    game_events.captured_supply = (game_stats.event_data >> 2) & 1;
    game_events.captured_central_buff = (game_stats.event_data >> 30) & 1;
    game_events.opponent_captured_central_buff = (game_stats.event_data >> 31) & 1;
    ROS_INFO("Captured supply: %d", game_events.captured_supply);
    ROS_INFO("Captured central buff: %d", game_events.captured_central_buff);
    ROS_INFO("Opponent captured central buff: %d", game_events.opponent_captured_central_buff);
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

    int hp_before_returning;
    if (!n.getParam("hp_before_returning", hp_before_returning)) {
        ROS_ERROR("Failed to get param 'hp_before_returning'");
    }

    std::vector<double> base_location;
    if (!n.getParam("base_location", base_location)) {
        ROS_ERROR("Failed to get param 'base_location'");
    }

    std::vector<double> supplies_location;
    if (!n.getParam("supplies_location", supplies_location)) {
        ROS_ERROR("Failed to get param 'supplies_location'");
    }

    pub = n.advertise<decision::TargetCommand>("/decision/target_cmd", 10);

    ros::Subscriber armors_sub = n.subscribe("/detector/armors", 10, armorCallback);

    ros::Subscriber gamestats_sub = n.subscribe("/decision/gamestats", 10, gamestatsCallback);

    NavClient nav_client("nav_ctrl", true);

    nav_client.waitForServer();

    DecisionState decision_state = DecisionState::PATROLLING;
    int current_waypoint = 0;
    bool waypoint_changed = true;
    bot_sim::NavActionGoal goal;

    while (ros::ok()) {
        if (decision_state == DecisionState::PATROLLING && waypoint_changed == true) {
            goal.goal_x = patrol_path[current_waypoint][0];
            goal.goal_y = patrol_path[current_waypoint][1];
            nav_client.sendGoal(goal);
            waypoint_changed = false;
        }

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

        if (decision_state == DecisionState::PATROLLING || decision_state == DecisionState::TRACKING) {
            if (target.tracking) {
                cmd_msg.id = std::stoi(target.id);
                decision_state = DecisionState::TRACKING;
                goal.goal_x = target.position.x;
                goal.goal_y = target.position.y;
                nav_client.sendGoal(goal);
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
            else {
                decision_state = DecisionState::PATROLLING;
                current_waypoint = getNearestWaypoint(patrol_path);
                waypoint_changed = true;
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

        if (decision_state == DecisionState::PATROLLING && nav_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            waypoint_changed = true;
            current_waypoint = (current_waypoint + 1) % patrol_path.size();
        }

        ros::spinOnce();
    }

    return 0;
}
