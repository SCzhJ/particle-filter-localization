#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped

vel = Twist()

def GetVel(msg):
    global vel
    vel = msg
    vel.angular.z = -vel.angular.z
    vel.linear.x  = vel.linear.x
    vel.linear.y  = vel.linear.y

if __name__=="__main__":
    rospy.init_node("cmd_vel_converter")
    sub = rospy.Subscriber("cmd_vel", Twist, GetVel, queue_size=100)
    pub = rospy.Publisher("velocity", Twist, queue_size=100)
    deltaTime = 0.01
    rate = rospy.Rate(1/deltaTime)

    while not rospy.is_shutdown():
        pub.publish(vel)
        rate.sleep()