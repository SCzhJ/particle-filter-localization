#!/usr/bin/python3

import rospy
from sensor_msgs.msg import LaserScan


new_msg = LaserScan()
def callback(msg):
    global new_msg
    new_msg = msg
    new_msg.intensities = []


if __name__=="__main__":
    rospy.init_node("cut_scan")
    sub = rospy.Subscriber("scan_laser", LaserScan, callback, queue_size=10)
    pub = rospy.Publisher("scan", LaserScan, queue_size=10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        new_msg.header.stamp = rospy.Time.now()
        pub.publish(new_msg)
        rate.sleep()

