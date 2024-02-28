#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <serial/serial.h>
#include <algorithm>
#include <string>

serial::Serial ser;

union FloatToByte{
    float f;
    uint8_t bytes[sizeof(float)];
};

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
	ROS_INFO_STREAM("Serial Port initialized");
    uint8_t buffer[13];

    // Start of Frame
    buffer[0] = 0x4A; // SOF

    // Linear velocities x
    FloatToByte linear_x;
    linear_x.f = msg->linear.x;
    std::copy(std::begin(linear_x.bytes),std::end(linear_x.bytes),&buffer[1]);

    // Linear velocities y
    FloatToByte linear_y;
    linear_y.f = msg->linear.y;
    std::copy(std::begin(linear_y.bytes),std::end(linear_y.bytes),&buffer[5]);

    // Angular velocity
    FloatToByte angular_z;
    angular_z.f = msg->angular.z;
    std::copy(std::begin(angular_z.bytes),std::end(angular_z.bytes),&buffer[9]);

    // Write to the serial port
    // ser.write(0xA3, 1); // test message
    size_t bytes_written = ser.write(buffer,13);
    if (bytes_written < 13){
        ROS_ERROR("Failed to write all bytes to the serial port");
    }

	 // Logging buffer content for debug
	 std::stringstream ss;
	 for(int i=0;i<13;i++){
		 ss << "0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(buffer[i]) << " ";
	 }
	 ROS_INFO_STREAM("Buffer content: " << ss.str());

}

int main(int argc, char **argv)
{
    std::string node_name = "cmd2ser";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    std::string base_serial_port;
    if (nh.getParam("/"+node_name+"/base_serial_port",base_serial_port)){
		ROS_INFO("Got param: %s", base_serial_port.c_str());
    }
    else{
    	ROS_ERROR("Failed to get param: %s", base_serial_port.c_str());
    }

    std::string vel_topic;
    if (nh.getParam("/"+node_name+"/vel_topic",vel_topic)){
		ROS_INFO("Got param: %s", vel_topic.c_str());
    }
    else{
    	ROS_ERROR("Failed to get param: %s", vel_topic.c_str());
    }

    try{
	    // Configuration
    	ser.setPort(base_serial_port);
    	ser.setBaudrate(115200);
    	serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    	ser.setTimeout(to);
		ser.setBytesize(serial::eightbits);
		//ser.setStopbits(serial::stopbits_one);
    	ser.setParity(serial::parity_none);
		ser.setFlowcontrol(serial::flowcontrol_none);

	    // open port
    	ser.open();
    }
    catch(serial::IOException& e){
		ROS_ERROR_STREAM("Unable to open port");
		return -1;
    }

    if(ser.isOpen()){
		ROS_INFO_STREAM("Serial Port initialized");
    }
    else{
		return -1;
    }

    ros::Subscriber sub = nh.subscribe(vel_topic, 1000, cmdVelCallback);
	ROS_INFO_STREAM("Serial Port initialized");

    ros::spin();

    return 0;
}
