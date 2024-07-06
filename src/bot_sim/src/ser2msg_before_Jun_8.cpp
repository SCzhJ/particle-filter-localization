#include <ros/ros.h>
#include <serial/serial.h>
#include <geometry_msgs/Twist.h>
#include <algorithm>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

const double PI = 3.14159265358979323846;

serial::Serial ser;
const int write_length = 13;
const int read_length = 10;

geometry_msgs::TransformStamped transformRotbaseToVirtual;
geometry_msgs::TransformStamped transformGimbalToRotbase;
geometry_msgs::TransformStamped transformMapToGimbal;
geometry_msgs::TransformStamped loc;


union FloatToByte{
    float f;
    uint8_t bytes[sizeof(float)];
};

class Message {
public:
    static const uint8_t SOF = 0xFF;
    static const uint8_t eof = 0xFE;
    // float imu_change_threshold = 0.2;
    // float relative_change_threshold = 15;
    // float past_imu_angle;
    // float past_relative_angle;
    float imu_angle;
    float relative_angle;
    bool readFromBuffer(std::vector<uint8_t>& buffer) {
        if (buffer.size() < read_length) {
            return false; // Not enough data, 
        }
        else if (buffer[0] == SOF && buffer[read_length-1] == eof) {
            memcpy(&this->relative_angle, &buffer[1], sizeof(float));
            memcpy(&this->imu_angle, &buffer[5], sizeof(float));
            buffer.erase(buffer.begin(), buffer.begin()+read_length);
            // ROS_INFO("Read from buffer");
            return true;
        }
        else {
            buffer.erase(buffer.begin());
            return true; // Format dismatch, drop the first byte and try again 
        }
    }
    void printData() {
        ROS_INFO_STREAM("imu_angle: " << this->imu_angle);
        ROS_INFO_STREAM("relative_angle: " << this->relative_angle);
    }
};

geometry_msgs::Twist cmd_vel;

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    cmd_vel = *msg;
    printf("msg_received");
    // std::cout<<cmd_vel.linear.x<<' '<<cmd_vel.linear.y<<std::endl;
}

int main(int argc, char** argv)
{
    std::string node_name = "ser2msg";
    ros::init(argc, argv, node_name);

    ros::NodeHandle nh;

    std::string serial_port;
    if (!nh.getParam("/"+node_name+"/serial_port", serial_port))
    {
        ROS_ERROR("Failed to retrieve parameter 'serial_port'");
        return -1;
    }
    float delta_time;
    if (!nh.getParam("/"+node_name+"/delta_time", delta_time))
    {
        ROS_ERROR("Failed to retrieve parameter 'delta_time'");
        return -1;
    }
    std::string virtual_frame;
    if (!nh.getParam("/"+node_name+"/virtual_frame", virtual_frame))
    {
        ROS_ERROR("Failed to retrieve parameter 'virtual_frame'");
        return -1;
    }
    std::string rotbase_frame;
    if (!nh.getParam("/"+node_name+"/rotbase_frame", rotbase_frame))
    {
        ROS_ERROR("Failed to retrieve parameter 'rotbase_frame'");
        return -1;
    }
    std::string gimbal_frame;
    if (!nh.getParam("/"+node_name+"/gimbal_frame", gimbal_frame))
    {
        ROS_ERROR("Failed to retrieve parameter 'gimbal_frame'");
        return -1;
    }
    std::string vel_topic;
    if (!nh.getParam("/"+node_name+"/vel_topic",vel_topic)){
    	ROS_ERROR("Failed to get param: %s", vel_topic.c_str());
    }
    std::string _3DLidar_frame;
    if (!nh.getParam("/"+node_name+"/_3DLidar_frame", _3DLidar_frame))
    {
        ROS_ERROR("Failed to retrieve parameter '_3DLidar_frame'");
        return -1;
    }

    // message.setTransform(virtual_frame, rotbase_frame);

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

    ros::Subscriber sub = nh.subscribe(vel_topic, 1000, cmdVelCallback);

    tf2_ros::TransformBroadcaster tfb;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    
    tf2::Quaternion q1;
    transformRotbaseToVirtual.header.frame_id = rotbase_frame;
    transformRotbaseToVirtual.child_frame_id = virtual_frame;
    transformRotbaseToVirtual.transform.translation.x = 0.0;
    transformRotbaseToVirtual.transform.translation.y = 0.0;
    transformRotbaseToVirtual.transform.translation.z = -0.1;

    
    tf2::Quaternion q2;
    transformGimbalToRotbase.header.frame_id = gimbal_frame;
    transformGimbalToRotbase.child_frame_id = rotbase_frame;
    transformGimbalToRotbase.transform.translation.x = 0.0;
    transformGimbalToRotbase.transform.translation.y = 0.0;
    transformGimbalToRotbase.transform.translation.z = -0.3;

    Message message;
    std::vector<uint8_t> buffer_recv;
    uint8_t byte;
    uint8_t buffer_send[read_length];
    buffer_send[0] = 0x4A; // SOF
    ros::Rate rate = ros::Rate(1/delta_time);

    tf2::Transform gimbalframe;
    tf2::Transform rotbaseframe;
    tf2::Transform virtualframe;
    tf2::Transform location;

    loc.header.frame_id = "map";
    loc.child_frame_id = virtual_frame;

    while(ros::ok()){

        // Read data from the serial port
        buffer_recv.clear();
        size_t bytes_available = ser.available();
        if (bytes_available < read_length) {
            // ROS_WARN("Not enough data available from the serial port");
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

        // Write data to serial port
        // Linear velocities x
        FloatToByte linear_x;
        linear_x.f = cmd_vel.linear.x;
        std::copy(std::begin(linear_x.bytes),std::end(linear_x.bytes),&buffer_send[1]);
        // Linear velocities y
        FloatToByte linear_y;
        linear_y.f = cmd_vel.linear.y;
        std::copy(std::begin(linear_y.bytes),std::end(linear_y.bytes),&buffer_send[5]);
        // Angular velocity
        FloatToByte angular_z;
        angular_z.f = cmd_vel.angular.z;
        std::copy(std::begin(angular_z.bytes),std::end(angular_z.bytes),&buffer_send[9]);

        // Write to the serial port
        size_t bytes_written = ser.write(buffer_send,write_length);
        if (bytes_written < write_length){
            ROS_ERROR("Failed to write all bytes to the serial port");
        }

        q1.setRPY(0,0,-message.imu_angle);
        transformRotbaseToVirtual.transform.rotation.x = q1.x();
        transformRotbaseToVirtual.transform.rotation.y = q1.y();
        transformRotbaseToVirtual.transform.rotation.z = q1.z();
        transformRotbaseToVirtual.transform.rotation.w = q1.w();
        transformRotbaseToVirtual.header.stamp = ros::Time::now();
        // tfb.sendTransform(transformRotbaseToVirtual);

        q2.setRPY(0,0,-message.relative_angle);
        transformGimbalToRotbase.transform.rotation.x = q2.x();
        transformGimbalToRotbase.transform.rotation.y = q2.y();
        transformGimbalToRotbase.transform.rotation.z = q2.z();
        transformGimbalToRotbase.transform.rotation.w = q2.w();
        transformGimbalToRotbase.header.stamp = ros::Time::now();
        // tfb.sendTransform(transformGimbalToRotbase);

        try{
            transformMapToGimbal = tfBuffer.lookupTransform("map", "gimbal_frame",ros::Time(0),ros::Duration(5.0));
        }
        catch(tf2::TransformException &ex){
            ROS_WARN("%s",ex.what());
        }
        tf2::fromMsg(transformMapToGimbal.transform, gimbalframe);
        tf2::fromMsg(transformGimbalToRotbase.transform,rotbaseframe);
        tf2::fromMsg(transformRotbaseToVirtual.transform,virtualframe);
        // location = gimbalframe;
        // // * rotbaseframe * virtualframe;
        location = gimbalframe * rotbaseframe * virtualframe;
        loc.transform = tf2::toMsg(location);
        loc.header.stamp = ros::Time::now();
        tfb.sendTransform(loc);
        ros::spinOnce();
        rate.sleep();
    }
    ros::spin();
    return 0;
}