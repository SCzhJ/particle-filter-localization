#!/usr/bin/env python3
import rospy
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
if __name__ == '__main__':
    try:
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.init_node('cmd_vel_publisher', anonymous=True)
        rate = rospy.Rate(1) # 10hz
        msg=Twist()
        while not rospy.is_shutdown():
            msg.linear.x = 0.1
            msg.linear.y = 0
            msg.angular.z = 0
            # rospy.loginfo(str(msg))
            pub.publish(msg)
    except rospy.ROSInterruptException:
        pass
    finally:
        msg.linear.x = 0
        msg.linear.y = 0
        msg.angular.z = 0
        pub.publish(msg)

