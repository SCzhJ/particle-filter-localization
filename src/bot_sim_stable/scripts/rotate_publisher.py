#!/usr/bin/env python3
import rospy
import time
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry

def talker():
    pub = rospy.Publisher('/rotate', Float64, queue_size=10)
    rospy.init_node('rotate_publisher', anonymous=True)
    # rospy.loginfo(str(msg))
    while not rospy.is_shutdown():
        pub.publish(2)
    rospy.spin()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
