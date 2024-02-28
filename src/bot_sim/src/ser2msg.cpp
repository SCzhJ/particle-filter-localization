#include <ros/ros.h>
#include <serial/serial.h>
#include <geometry_msgs/Twist.h>

serial::Serial ser;

void receiveCallback(const ros::TimerEvent& event)
{
    // Read data from the serial port
    if (ser.available())
    {
	std::vector<uint8_t> buffer(13);
	ser.read(&buffer[0],13);

        // Process received data
        if (buffer.size() == 13 && buffer[0] == 0xA4)
        {
            // Extract linear velocities x
            float linear_x;
            memcpy(&linear_x, &buffer[1], sizeof(float));

            // Extract linear velocities y
            float linear_y;
            memcpy(&linear_y, &buffer[5], sizeof(float));

            // Extract angular velocity
            float angular_z;
            memcpy(&angular_z, &buffer[9], sizeof(float));

            // Print received values
            ROS_INFO("Received data: Linear_x=%.2f, Linear_y=%.2f, Angular_z=%.2f",
                     linear_x, linear_y, angular_z);

        }
	else{
            ROS_ERROR_STREAM("buffer size shorter than 13");
	}
    }
}

int main(int argc, char** argv)
{
    std::string node_name = "ser2msg";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    std::string receive_serial_port;
    if (!nh.getParam("/" + node_name + "/receive_serial_port", receive_serial_port))
    {
        ROS_ERROR("Failed to retrieve parameter 'serial_port'");
        return -1;
    }

    try
    {
        // Configure the serial port
        ser.setPort(receive_serial_port);
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.setBytesize(serial::eightbits);
        //ser.setStopbits(serial::stopbits_one);
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

    // Set up a timer to receive messages periodically
    ros::Timer timer = nh.createTimer(ros::Duration(0.1), receiveCallback);

    ros::spin();

    return 0;
}
