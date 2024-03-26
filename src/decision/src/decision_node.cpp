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

std::vector<decision::Armor> armors;
decision::Target target;
decision::GameStats game_stats;
geometry_msgs::Point position;
int last_hp;

typedef actionlib::SimpleActionClient<bot_sim::NavActionAction> NavClient;

enum class DecisionState {
    CHARGING,
    RETREAT,
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

std::chrono::steady_clock::time_point last_healing_time = std::chrono::steady_clock::now();
bool at_supplies_location = false;
int hp_restored = 0;

bool healingAvailable() {
    auto now = std::chrono::steady_clock::now();
    auto healing_timeout = std::chrono::duration_cast<std::chrono::seconds>(now - last_healing_time).count();
    return game_stats.stage_remain_time >= 60 && hp_restored < 500 && (at_supplies_location && healing_timeout < 6 || !at_supplies_location);
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

    int hp_before_retreat;
    if (!n.getParam("hp_before_retreat", hp_before_retreat)) {
        ROS_ERROR("Failed to get param 'hp_before_retreat'");
    }

    int hp_before_healing;
    if (!n.getParam("hp_before_healing", hp_before_healing)) {
        ROS_ERROR("Failed to get param 'hp_before_healing'");
    }

    int restore_to_hp;
    if (!n.getParam("restore_to_hp", restore_to_hp)) {
        ROS_ERROR("Failed to get param 'restore_to_hp'");
    }

    double max_nav_succeed_time;
    if (!n.getParam("max_nav_succeed_time", max_nav_succeed_time)) {
        ROS_ERROR("Failed to get param 'max_nav_succeed_time'");
    }

    XmlRpc::XmlRpcValue base_location_list;
    std::vector<double> base_location;
    if (!n.getParam("base_location", base_location_list)) {
        ROS_ERROR("Failed to get param 'base_location'");
    }

    XmlRpc::XmlRpcValue central_point_location_list;
    std::vector<double> central_point_location;
    if (!n.getParam("central_point_location", central_point_location_list)) {
        ROS_ERROR("Failed to get param 'central_point_location'");
    }

    ros::Subscriber armors_sub = n.subscribe("/detector/armors", 10, armorCallback);

    ros::Subscriber gamestats_sub = n.subscribe("/decision/gamestats", 10, gamestatsCallback);

    ros::Subscriber pose_sub = n.subscribe("/amcl_pose", 10, poseCallback);

    NavClient nav_client("nav_ctrl", true);

    std::cout << "Initialization complete. Waiting for navigation." << std::endl;

    nav_client.waitForServer();

    DecisionState decision_state = DecisionState::CHARGING;
    int current_waypoint = 2;
    int last_hp = 600;
    bool waypoint_changed = true;
    bot_sim::NavActionGoal goal;
    geometry_msgs::PoseWithCovarianceStamped pose_msg;
    std::chrono::steady_clock::time_point last_goal_time = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point last_nav_succeed_time = std::chrono::steady_clock::now();

    std::cout << "Got navigation server. Starting decision loop." << std::endl;

    while (ros::ok()) {
        if (game_stats.game_progress != 4) {
            ros::spinOnce();
            continue;
        }

        auto now = std::chrono::steady_clock::now();

        if (waypoint_changed == true) {
            goal.goal_x = patrol_path[current_waypoint][0];
            goal.goal_y = patrol_path[current_waypoint][1];
            std::cout << "Sending goal: (" << goal.goal_x << ", " << goal.goal_y << ")" << std::endl;
            std::cout << "Current position: (" << position.x << ", " << position.y << ")" << std::endl;
            nav_client.sendGoal(goal);
            waypoint_changed = false;
            last_goal_time = now;
        }

        auto send_goal_time_diff = std::chrono::duration_cast<std::chrono::seconds>(now - last_goal_time);

        if (send_goal_time_diff.count() >= 5) {
            std::cout << "Resending goal: (" << goal.goal_x << ", " << goal.goal_y << ")" << std::endl;
            std::cout << "Current position: (" << position.x << ", " << position.y << ")" << std::endl;
            nav_client.sendGoal(goal);
            last_goal_time = now;
        }

        if (game_stats.remain_hp > last_hp && last_hp != 0) {
            last_healing_time = now;
            hp_restored += 100;
        }

        last_hp = game_stats.remain_hp;

        if (decision_state != DecisionState::HEALING && decision_state != DecisionState::RETURNING && game_stats.remain_hp < hp_before_healing) {
            if (healingAvailable()) {
                decision_state = DecisionState::HEALING;
                current_waypoint = std::max(current_waypoint - 1, 0);
                waypoint_changed = true;
            }
            else {
                decision_state = DecisionState::RETURNING;
                current_waypoint = std::max(current_waypoint - 1, 1);
                waypoint_changed = true;
            }
        } else if (decision_state == DecisionState::CHARGING && game_stats.remain_hp < hp_before_retreat) {
            decision_state = DecisionState::RETREAT;
            current_waypoint = std::max(current_waypoint - 1, 2);
            waypoint_changed = true;
        }

        if (decision_state == DecisionState::HEALING && game_stats.remain_hp >= restore_to_hp) {
            decision_state = DecisionState::CHARGING;
            current_waypoint = std::min(current_waypoint + 1, 3);
            waypoint_changed = true;
            at_supplies_location = false;
        }

        if (decision_state == DecisionState::HEALING && !healingAvailable()) {
            decision_state = DecisionState::RETURNING;
            current_waypoint = std::max(current_waypoint - 1, 1);
            waypoint_changed = true;
            at_supplies_location = false;
        }

        if (game_stats.stage_remain_time <= 240) {
            patrol_path[3] = central_point_location;
            if (decision_state == DecisionState::CHARGING)
                waypoint_changed = true;
        }

        // if ((decision_state == DecisionState::RETURNING || decision_state == DecisionState::RETREAT) && game_stats.remain_hp >= restore_to_hp) {
        //     decision_state = DecisionState::CHARGING;
        //     current_waypoint = std::min(current_waypoint + 1, 3);
        //     waypoint_changed = true;
        // }

        bool nav_succeeded = (nav_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) && !waypoint_changed;

        if (nav_succeeded) {
            last_nav_succeed_time = now;
        }

        // difference between current time and last nav succeed time
        auto nav_succeed_timeout = std::chrono::duration_cast<std::chrono::seconds>(now - last_nav_succeed_time).count();
        // if (nav_succeed_timeout > max_nav_succeed_time) {
        //     decision_state = DecisionState::RETURNING;
        //     current_waypoint = 1;
        //     waypoint_changed = true;
        // }

        if (decision_state == DecisionState::CHARGING && nav_succeeded && current_waypoint < 3) {
            waypoint_changed = true;
            current_waypoint += 1;
        }

        if (decision_state == DecisionState::RETREAT && nav_succeeded && current_waypoint > 2) {
            waypoint_changed = true;
            current_waypoint = 2;
        }

        if (decision_state == DecisionState::RETURNING && nav_succeeded && current_waypoint > 1) {
            waypoint_changed = true;
            current_waypoint -= 1;
        }

        if (decision_state == DecisionState::HEALING && nav_succeeded) {
            if (current_waypoint == 0) {
                last_healing_time = std::chrono::steady_clock::now();
                at_supplies_location = true;
            }
            else {
                waypoint_changed = true;
                current_waypoint -= 1;
            }
        }

        ros::spinOnce();
    }

    return 0;
}
