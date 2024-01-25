#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from math import isnan

class LaserScanProcessor:
    def __init__(self, angle_min, angle_max):
        self.angle_min = angle_min
        self.angle_max = angle_max
        self.publisher = rospy.Publisher('/filtered_scan', LaserScan, queue_size=10)
        self.subscriber = rospy.Subscriber('/scan', LaserScan, self.callback)

    def callback(self, msg):
        filtered_scan = LaserScan()
        filtered_scan.header = msg.header
        filtered_scan.angle_min = msg.angle_min
        filtered_scan.angle_max = msg.angle_max
        filtered_scan.angle_increment = msg.angle_increment
        filtered_scan.time_increment = msg.time_increment
        filtered_scan.scan_time = msg.scan_time
        filtered_scan.range_min = msg.range_min
        filtered_scan.range_max = msg.range_max

        angle = msg.angle_min
        for r in msg.ranges:
            if self.angle_min >= angle or angle >= self.angle_max and not isnan(r):
                filtered_scan.ranges.append(r)
            else:
                filtered_scan.ranges.append(float('Inf'))
            angle += msg.angle_increment

        self.publisher.publish(filtered_scan)

if __name__ == '__main__':
    rospy.init_node('laser_scan_angle_2d')
    processor = LaserScanProcessor(-1.3, 1.3)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        rospy.spin()
        rate.sleep()