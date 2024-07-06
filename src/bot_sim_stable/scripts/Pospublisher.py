#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

def callback(data):
    rospy.loginfo("Received message: %s", data)
    print("sdfsdf")

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/robot_pose_ekf/odom_combined", PoseWithCovarianceStamped, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()