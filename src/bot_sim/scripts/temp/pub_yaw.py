#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

def callback(msg):
    # Extract the orientation quaternion from the message
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

    # Convert the quaternion to Euler angles
    _, _, yaw = euler_from_quaternion(orientation_list)

    rospy.loginfo("Yaw: %f", yaw)

def main():
    rospy.init_node('yaw_printer')

    # Subscribe to the odometry topic
    rospy.Subscriber("/aft_mapped_to_init", Odometry, callback)

    # Spin until the node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()