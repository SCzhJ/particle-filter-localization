#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <vector>
#include <Eigen/Core>

const double PI = 3.14159265358979323846;

class ImuOdometryCalculator {
public:
    ImuOdometryCalculator() : is_calibrated(false) {
        // Initialize the node handle
        ros::NodeHandle nh;

        // Subscriber to the IMU data topic
        imu_sub = nh.subscribe("/IMU_data", 50, &ImuOdometryCalculator::imuCallback, this);

        // Publisher for the odometry data
        odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);

        // Initialize position, velocity, and bias
        x = y = z = vx = vy = vz = ax_bias = ay_bias = az_bias = 0.0;

        // Initialize the filter coefficients
        double wc = 2 * PI * cutoff_freq;
        double T = 1 / sampling_freq;
        double tan_wc_T_2 = std::tan(wc * T / 2);
        a0 = 1 + std::sqrt(2) * tan_wc_T_2 + tan_wc_T_2 * tan_wc_T_2;
        a1 = -2 * (1 - tan_wc_T_2 * tan_wc_T_2) / a0;
        a2 = (1 - std::sqrt(2) * tan_wc_T_2 + tan_wc_T_2 * tan_wc_T_2) / a0;
        b0 = tan_wc_T_2 * tan_wc_T_2 / a0;
        b1 = 2 * b0;
        b2 = b0;

        // Normalize the 'a' coefficients
        a0 = 1;

        ROS_INFO("Coefficients: a0: %f, a1: %f, a2: %f, b0: %f, b1: %f, b2: %f", a0, a1, a2, b0, b1, b2);
        ROS_INFO("Wait for IMU calibration...");

    }

    bool isCalibrated() {
        return is_calibrated;
    }

    double SecondOrderLowPassFilter(const double& x, const double& x1, const double& x2, const double& y1, const double& y2){
        return b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
    }
    double WeightedAccelerationSum(const double& x, const double& x1, const double& x2, const double& y1, const double& y2) {
        return 0.35 * x + 0.2 * x1 + 0.1 * x2 + 0.2 * y1 + 0.05 * y2;
    }

    double WeightedVelocitySum(const double& v, const double& prev_v, const double& prev_prev_v){
        return 0.65 * v + 0.3 * prev_v + 0.05 * prev_prev_v;
    }

    void publishOdometry() {
        // Create a rate object with a frequency of 50Hz
        ros::spinOnce();
        ros::WallRate loop_rate(cutoff_freq);
        ROS_INFO("Publishing odometry data...");
        while (ros::ok()) {
            // Sleep for the rest of the cycle, to maintain the desired rate
            for(int i = 0; i < sampling_freq/cutoff_freq; i++)
                ros::spinOnce();

            loop_rate.sleep();

            // Calculate time difference
            ros::Time current_time = ros::Time::now();
            double dt = (current_time - last_time).toSec();

            // Calculate the filtered acceleration
            filtered_acceleration.x = WeightedAccelerationSum(current_sample.linear_acceleration.x, prev_acceleration.x, prev_prev_acceleration.x, prev_filtered_acceleration.x, prev_prev_filtered_acceleration.x);
            filtered_acceleration.y = WeightedAccelerationSum(current_sample.linear_acceleration.y, prev_acceleration.y, prev_prev_acceleration.y, prev_filtered_acceleration.y, prev_prev_filtered_acceleration.y);
            filtered_acceleration.z = WeightedAccelerationSum(current_sample.linear_acceleration.z, prev_acceleration.z, prev_prev_acceleration.z, prev_filtered_acceleration.z, prev_prev_filtered_acceleration.z);

            // Save the filtered acceleration for the next iteration
            prev_prev_filtered_acceleration = prev_filtered_acceleration;
            prev_filtered_acceleration = filtered_acceleration;

            // Save the velocity for the next iteration
            prev_prev_vel = prev_vel;
            prev_vel.x = vx;
            prev_vel.y = vy;
            prev_vel.z = vz;

            // Integrate the IMU linear acceleration data
            vx += filtered_acceleration.x * dt - vx_bias;
            vy += filtered_acceleration.y * dt - vy_bias;
            vz += filtered_acceleration.z * dt - vz_bias;

            vx = WeightedVelocitySum(vx, prev_vel.x, prev_prev_vel.x);
            vy = WeightedVelocitySum(vy, prev_vel.y, prev_prev_vel.y);
            vz = WeightedVelocitySum(vz, prev_vel.z, prev_prev_vel.z);

            // // update velocity bias
            // if (std::abs(vx) < vel_noise_threshold && \
            //     std::abs(prev_vel.x) < vel_noise_threshold && \
            //     std::abs(prev_prev_vel.x) < vel_noise_threshold)
            //     vx_bias += (vx + prev_vel.x + prev_prev_vel.x) / 3;
            // if (std::abs(vy) < vel_noise_threshold && \
            //     std::abs(prev_vel.y) < vel_noise_threshold && \
            //     std::abs(prev_prev_vel.y) < vel_noise_threshold)
            //     vy_bias += (vy + prev_vel.y + prev_prev_vel.y) / 3;
            // if (std::abs(vz) < vel_noise_threshold && \
            //     std::abs(prev_vel.z) < vel_noise_threshold && \
            //     std::abs(prev_prev_vel.z) < vel_noise_threshold)
            //     vz_bias += (vz + prev_vel.z + prev_prev_vel.z) / 3;

            // Update position based on velocity
            x += vx * dt;
            y += vy * dt;
            z += vz * dt;

            // Create and publish the odometry message
            nav_msgs::Odometry odom;
            odom.header.stamp = current_time;
            odom.header.frame_id = "odom";

            // Set the position
            odom.pose.pose.position.x = x;
            odom.pose.pose.position.y = y;
            odom.pose.pose.position.z = z;

            // Set the orientation
            odom.pose.pose.orientation = current_sample.orientation;

            // Set the velocity
            odom.child_frame_id = "base_link";
            odom.twist.twist.linear.x = vx;
            odom.twist.twist.linear.y = vy;
            odom.twist.twist.linear.z = vz;

            // Set the angular velocity
            odom.twist.twist.angular.x = current_sample.angular_velocity.x;
            odom.twist.twist.angular.y = current_sample.angular_velocity.y;
            odom.twist.twist.angular.z = current_sample.angular_velocity.z;

            // Publish the message
            odom_pub.publish(odom);

            // Save last time
            last_time = current_time;
        }
    }

    void calibrateImuBias(const sensor_msgs::Imu::ConstPtr& msg) {
        // Collect a number of samples to calculate the bias
        if (static_cast<int>(accel_samples.size()) < calibration_samples) {
            accel_samples.push_back(*msg);
            return;
        }

        // Calculate average bias from collected samples
        for (const auto& sample : accel_samples) {
            ax_bias += sample.linear_acceleration.x;
            ay_bias += sample.linear_acceleration.y;
            az_bias += sample.linear_acceleration.z;
        }
        
        ax_bias /= calibration_samples;
        ay_bias /= calibration_samples;
        az_bias /= calibration_samples;

        ROS_INFO("IMU Calibration complete. Biases - ax: %f, ay: %f, az: %f", ax_bias, ay_bias, az_bias);

        is_calibrated = true;
        accel_samples.clear();
        last_time = ros::Time::now();
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        if (!is_calibrated) {
            calibrateImuBias(msg);
            return;
        }
        prev_prev_acceleration = prev_acceleration;
        prev_acceleration = current_sample.linear_acceleration;
        current_sample = *msg;
        current_sample.linear_acceleration.x -= ax_bias;
        current_sample.linear_acceleration.y -= ay_bias;
        current_sample.linear_acceleration.z -= az_bias;
        if (std::abs(current_sample.linear_acceleration.x) < acc_noise_threshold && \
            std::abs(prev_acceleration.x) < acc_noise_threshold && \
            std::abs(prev_prev_acceleration.x) < acc_noise_threshold)
            current_sample.linear_acceleration.x = 0;
            ax_bias += (current_sample.linear_acceleration.x + prev_acceleration.x + prev_prev_acceleration.x) / 3;
        if (std::abs(current_sample.linear_acceleration.y) < acc_noise_threshold && \
            std::abs(prev_acceleration.y) < acc_noise_threshold && \
            std::abs(prev_prev_acceleration.y) < acc_noise_threshold)
            current_sample.linear_acceleration.y = 0;
            ay_bias += (current_sample.linear_acceleration.y + prev_acceleration.y + prev_prev_acceleration.y) / 3;
        if (std::abs(current_sample.linear_acceleration.z) < acc_noise_threshold && \
            std::abs(prev_acceleration.z) < acc_noise_threshold && \
            std::abs(prev_prev_acceleration.z) < acc_noise_threshold)
            current_sample.linear_acceleration.z = 0;
            az_bias += (current_sample.linear_acceleration.z + prev_acceleration.z + prev_prev_acceleration.z) / 3;
        return;
    }

private:
    ros::Subscriber imu_sub;
    ros::Publisher odom_pub;
    ros::Time last_time;

    // Position and velocity
    double x, y, z, vx, vy, vz;

    // Bias for the IMU acceleration
    double ax_bias, ay_bias, az_bias;
    double vx_bias, vy_bias, vz_bias;

    // Samples for calibration
    bool is_calibrated;
    std::vector<sensor_msgs::Imu> accel_samples;
    sensor_msgs::Imu current_sample;
    sensor_msgs::Imu prev_sample;
    const int calibration_samples = 1000; // Number of samples used for calibration

    // Filter parameters
    const int sampling_freq = 100;
    const int cutoff_freq = 25;
    const double alpha = 0.1;
    geometry_msgs::Vector3 filtered_acceleration;
    geometry_msgs::Vector3 prev_filtered_acceleration;
    geometry_msgs::Vector3 prev_prev_filtered_acceleration;
    geometry_msgs::Vector3 prev_acceleration;
    geometry_msgs::Vector3 prev_prev_acceleration;

    // Filter coefficients
    double a0, a1, a2, b0, b1, b2;

    // noise processing
    const double acc_noise_threshold = 0.006;
    const double vel_noise_threshold = 0.04;
    geometry_msgs::Vector3 prev_vel;
    geometry_msgs::Vector3 prev_prev_vel;
};

int main(int argc, char** argv) {
    // Initialize the ROS system and becomea node.
    ros::init(argc, argv, "imu_odometry_calculator");

    // Create an instance of the ImuOdometryCalculator
    ImuOdometryCalculator imu_odometry_calculator;
    while (!imu_odometry_calculator.isCalibrated()) {
        ros::spinOnce();
    }

    // Publish the odometry data
    imu_odometry_calculator.publishOdometry();

    return 0;
}