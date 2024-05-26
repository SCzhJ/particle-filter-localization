#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf2_ros
import threading
import subprocess

class AMCLMonitor:
    def __init__(self, timeout=10.0):
        self.timeout = timeout
        self.last_pose = None
        self.last_received = rospy.get_rostime()
        self.timer = threading.Timer(self.timeout, self.check_timeout)
        self.amcl_pose_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
        
        # Subscribe to the amcl_pose topic
        rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.pose_callback)
        self.timer.start()

    def pose_callback(self, msg):
        self.last_pose = msg
        self.last_received = rospy.get_rostime()
        # Reset the timer
        self.timer.cancel()
        self.timer = threading.Timer(self.timeout, self.check_timeout)
        self.timer.start()

    def check_timeout(self):
        # Time since the last received message
        time_since_last_received = (rospy.get_rostime() - self.last_received).to_sec()
        if time_since_last_received >= self.timeout:
            rospy.loginfo(f"No amcl_pose messages for {self.timeout} seconds. Restarting AMCL node.")
            self.restart_amcl()
        else:
            rospy.loginfo("amcl_pose is active.")

    def restart_amcl(self):
        # Restart the AMCL node
        # This is a placeholder command, replace it with the actual command you use to run AMCL
        subprocess.call(['rosnode', 'kill', '/amcl'])
        subprocess.call(['roslaunch', 'bot_sim', 'launch_amcl_real.launch'])
        # Wait for the node to start publishing again
        rospy.wait_for_message('amcl_pose', PoseWithCovarianceStamped, timeout=10)
        # Republish the last known pose
        if self.last_pose:
            self.amcl_pose_pub.publish(self.last_pose)
            rospy.loginfo("Republished the last known amcl_pose.")
        # Restart the timer
        self.timer = threading.Timer(self.timeout, self.check_timeout)
        self.timer.start()

if __name__ == '__main__':
    rospy.init_node('amcl_monitor')
    
    timeout_duration = rospy.get_param("~timeout_duration", 12.0)  # in seconds

    amcl_monitor = AMCLMonitor(timeout=timeout_duration)
    rospy.spin()