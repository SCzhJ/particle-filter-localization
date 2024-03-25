#include <ros/ros.h>
#include <serial/serial.h>
#include <geometry_msgs/Twist.h>
#include <algorithm>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include "decision/Armor.h"
#include "decision/Armors.h"
#include "decision/Angles.h"
#include "decision/GameStats.h"
#include <iostream>

const double PI = 3.14159265358979323846;

serial::Serial ser;
const int read_length_navigation = 10;
const int read_length_decision = 49;

ros::Publisher angles_pub;
ros::Publisher gamestats_pub;

class Message {
public:
    static const uint8_t SOF_NAVIGATION = 0xFF;
    static const uint8_t SOF_GAMESTATS = 0xAA;
    static const uint8_t EOF_NAVIGATION = 0xFE;
    // float imu_change_threshold = 0.2;
    // float relative_change_threshold = 15;
    // float past_imu_angle;
    // float past_relative_angle;
    decision::Angles angles_msg;
    decision::GameStats gamestats_msg;

    bool readFromBuffer(std::vector<uint8_t>& buffer) {
        if (buffer.size() < read_length_navigation || (buffer.size() > 0 && buffer[0] == SOF_GAMESTATS && buffer.size() < read_length_decision)) {  
            return false; // Not enough data
        }
        else if (buffer[0] == SOF_NAVIGATION && buffer[read_length_navigation-1] == EOF_NAVIGATION) {
            memcpy(&this->angles_msg.relative_angle, &buffer[1], sizeof(float));
            memcpy(&this->angles_msg.imu_angle, &buffer[5], sizeof(float));
            // this->angles_msg.imu_angle += PI;
            buffer.erase(buffer.begin(), buffer.begin()+read_length_navigation);
            // ROS_INFO("Read from buffer");
            ROS_INFO("imu_angle: %f", this->angles_msg.imu_angle);
            ROS_INFO("relative_angle: %f", this->angles_msg.relative_angle);
            angles_pub.publish(angles_msg);
            // ROS_INFO("Read from buffer");
            return true;
        }
        else if (buffer[0] == SOF_GAMESTATS) {
            // std::cout << (int)buffer[0] << " " << (int)buffer[1] << " " << (int)buffer[2] << std::endl;
            ROS_INFO("Received game statistics");
            memcpy(&rx_struct_, buffer.data(), sizeof(GameStats_));
            gamestats_msg.header = rx_struct_.header;
            gamestats_msg.game_type = rx_struct_.game_type;
            gamestats_msg.game_progress = rx_struct_.game_progress;
            gamestats_msg.stage_remain_time = rx_struct_.stage_remain_time;

            gamestats_msg.red_1_hp = rx_struct_.red_1_hp;
            gamestats_msg.red_2_hp = rx_struct_.red_2_hp;
            gamestats_msg.red_3_hp = rx_struct_.red_3_hp;
            gamestats_msg.red_4_hp = rx_struct_.red_4_hp;
            gamestats_msg.red_5_hp = rx_struct_.red_5_hp;
            gamestats_msg.red_7_hp = rx_struct_.red_7_hp;
            gamestats_msg.blue_1_hp = rx_struct_.blue_1_hp;
            gamestats_msg.blue_2_hp = rx_struct_.blue_2_hp;
            gamestats_msg.blue_3_hp = rx_struct_.blue_3_hp;
            gamestats_msg.blue_4_hp = rx_struct_.blue_4_hp;
            gamestats_msg.blue_5_hp = rx_struct_.blue_5_hp;
            gamestats_msg.blue_7_hp = rx_struct_.blue_7_hp;

            gamestats_msg.red_base_hp = rx_struct_.red_base_hp;
            gamestats_msg.blue_base_hp = rx_struct_.blue_base_hp;

            gamestats_msg.remain_hp = rx_struct_.remain_hp;
            gamestats_msg.max_hp = rx_struct_.max_hp;
            gamestats_msg.max_power = rx_struct_.max_power;
            gamestats_msg.heat0_limit = rx_struct_.heat0_limit;
            gamestats_msg.heat0 = rx_struct_.heat0;
            gamestats_msg.power = rx_struct_.power;
            gamestats_msg.buffer = rx_struct_.buffer;
            gamestats_msg.bullet_speed = rx_struct_.bullet_speed;
            gamestats_msg.shooter_speed_limit = rx_struct_.shooter_speed_limit;
            gamestats_msg.event_data = rx_struct_.event_data;
            gamestats_pub.publish(gamestats_msg);

            // ROS_INFO("Read from buffer");
            // ROS_INFO("game_type: %d", gamestats_msg.game_type);
            // ROS_INFO("game_progress: %d", gamestats_msg.game_progress);
            // ROS_INFO("stage_remain_time: %d", gamestats_msg.stage_remain_time);

            buffer.erase(buffer.begin(), buffer.begin()+read_length_decision);
            return true;
        }
        else {
            buffer.erase(buffer.begin());
            return true; // Format dismatch, drop the first byte and try again 
        }
    }
    // void printData() {
    //     ROS_INFO_STREAM("imu_angle: " << this->imu_angle);
    //     ROS_INFO_STREAM("relative_angle: " << this->relative_angle);
    // }
private:
    struct GameStats_ {
        uint8_t header;
        uint8_t game_type; //比赛类型 UL, UC etc.
        uint8_t game_progress; //当前比赛阶段 0未开始 1准备 2自检 3五秒倒计时 4 比赛中 5结算
        uint16_t stage_remain_time; // 当前阶段剩余时间/秒

        uint16_t red_1_hp;
        uint16_t red_2_hp;
        uint16_t red_3_hp;
        uint16_t red_4_hp;

        uint16_t red_5_hp;
        uint16_t red_7_hp;
        uint16_t blue_1_hp;
        uint16_t blue_2_hp;

        uint16_t blue_3_hp;
        uint16_t blue_4_hp;
        uint16_t blue_5_hp;
        uint16_t blue_7_hp;

        uint16_t red_base_hp;
        uint16_t blue_base_hp;

        uint16_t remain_hp;
        uint16_t max_hp;
        uint16_t max_power;
        uint16_t heat0_limit; //枪口热量限制
        uint16_t heat0;//枪口当前热量
        float power;//当前底盘功率
        float buffer;//当前功率缓冲
        float bullet_speed;
        uint8_t shooter_speed_limit; // 子弹速度限制

        uint32_t event_data;
                /*
                0：未占领/未激活
                1：已占领/已激活
                bit 0-2：
                bit 0：己方补给站前补血点的占领状态，1 为已占领
                bit 1：己方补给站内部补血点的占领状态，1 为已占领
                bit 2：己方补给区的占领状态，1 为已占领（仅 RMUL 适用）
                bit 3-5：己方能量机关状态
                bit 3：己方能量机关激活点的占领状态，1 为已占领
                bit 4：己方小能量机关的激活状态，1 为已激活
                bit 5：己方大能量机关的激活状态，1 为已激活
                bit 6-11：己方高地占领状态
                bit 6-7：己方环形高地的占领状态，1 为被己方占领，2 为被对方占领
                bit 8-9：己方梯形高地的占领状态，1 为被己方占领，2 为被对方占领
                bit 10-11：己方梯形高地的占领状态，1 为被己方占领，2 为被对方占领
                bit 12-18：己方基地虚拟护盾的剩余值百分比（四舍五入，保留整数）
                bit 19-27：飞镖最后一次击中己方前哨站或基地的时间（0-420，开局默认为0）
                bit 28-29：飞镖最后一次击中己方前哨站或基地的具体目标，开局默认为 0，
                1 为击中前哨站，2 为击中基地固定目标，3 为击中基地随机目标
                bit 30-31：中心增益点的占领情况，0 为未被占领，1 为被己方占领，2 为被
                对方占领，3 为被双方占领。（仅 RMUL 适用）
                */
    } __attribute__((packed));         // To avoid memory alignment issues

    GameStats_ rx_struct_{};
    uint8_t rx_buffer_[49]{};
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "serial_test_receive");
    ros::NodeHandle nh;
    Message message;
    uint8_t byte;
    std::vector<uint8_t> buffer_recv;
    std::string serial_port = "/dev/ttyACM0";
    if (!nh.getParam("serial_port", serial_port)) {
        ROS_WARN("Failed to get param \"serial_port\", using default value \"/dev/ttyACM0\".");
    }
    angles_pub = nh.advertise<decision::Angles>("/angle_topic", 1000);
    gamestats_pub = nh.advertise<decision::GameStats>("/decision/gamestats", 1000);
    float delta_time = 0.1;
    try
    {
        // Configure the serial port
        ser.setPort(serial_port);
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(10000);
        ser.setTimeout(to);
        ser.setBytesize(serial::eightbits);
        // ser.setStopbits(serial::stopbits_one);
        ser.setParity(serial::parity_none);
        ser.setFlowcontrol(serial::flowcontrol_none);

        // Open the serial port
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open the serial port");
        return -1;
    }

    if (ser.isOpen())
    {
        ROS_INFO_STREAM("Serial port initialized");
    }
    else
    {
        return -1;
    }
    
    ros::Rate rate(1 / delta_time);
    while (ros::ok()) {
        // Read data from the serial port
        buffer_recv.clear();
        size_t bytes_available = ser.available();
        if (bytes_available < read_length_navigation) {
            ROS_WARN("Not enough data available from the serial port");
        }
        else{
            while (ser.available())
            {
                ser.read(&byte,1);
                buffer_recv.push_back(byte);
            }
            while (message.readFromBuffer(buffer_recv));
            // message.printData();
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}