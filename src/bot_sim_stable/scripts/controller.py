#!/usr/bin/python3

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class JoyClass:
    def __init__(self, scale=1.0, offset=0.0, deadband=0.1):
        rospy.init_node("omni_robot_node")
        # self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_callback)
        self.wheel1_pub = rospy.Publisher("/omni_robot/first_wheel_joint/command", Float64, queue_size=10)
        self.wheel2_pub = rospy.Publisher("/omni_robot/second_wheel_joint/command", Float64, queue_size=10)
        self.wheel3_pub = rospy.Publisher("/omni_robot/third_wheel_joint/command", Float64, queue_size=10)
        self.wheel4_pub = rospy.Publisher("/omni_robot/fourth_wheel_joint/command", Float64, queue_size=10)
        self.velocity_subscriber = rospy.Subscriber('/cmd_vel', Twist, self.velocity_callback)
        self.rate = rospy.Rate(10)
        self.x = 0
        self.y = 0
        self.z = 0
        self.wheel1_pub.publish(0)
        self.wheel2_pub.publish(0)
        self.wheel3_pub.publish(0)
        self.wheel4_pub.publish(0)

    def velocity_callback(self,msg):
        self.x = (msg.linear.x*(2**0.5)/2-msg.linear.y*(2**0.5)/2) / 0.011755188 / 0.01
        self.y = (msg.linear.x*(2**0.5)/2+msg.linear.y*(2**0.5)/2) / 0.011755188 / 0.01
        self.z = -msg.angular.z/0.17
        wheel2 = (self.x) + (self.z)
        wheel1 = (-self.y) + (self.z)
        wheel3 = (self.y) + (self.z)
        wheel4 = (-self.x) + (self.z)
        self.wheel1_pub.publish(wheel1)
        self.wheel2_pub.publish(wheel2)
        self.wheel3_pub.publish(wheel3)
        self.wheel4_pub.publish(wheel4)

if __name__=="__main__":
    try:
        JoyClass()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
