#!/usr/bin/python3

from nav_msgs.msg import Odometry
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
class JoyClass:
    def __init__(self, scale=1.0, offset=0.0, deadband=0.1):
        rospy.init_node("plain_controller_node")
        self.joy_sub = rospy.Subscriber("/cmd_vel", Twist, self.velocity_callback)
        self.now = 0
        self.arg=0
        self.last_time=0
        self.rate=rospy.Rate(10)
        self.front_plain_pub = rospy.Publisher("/omni_robot/front_plain_joint/command", Float64, queue_size=10)
        self.velocity_sub=rospy.Subscriber("/odom", Odometry, self.callback)
        self.rate = rospy.Rate(10) # 10hz
    def velocity_callback(self,msg):
        self.now = msg.angular.z
    def callback(self,msg):
        angular_velocity = msg.twist.twist.angular
        self.arg=angular_velocity.z
    def main(self):
        while not rospy.is_shutdown():
            self.front_plain_pub.publish(self.now-self.arg)

if __name__=="__main__":
    try:
        Main=JoyClass()
        Main.main()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass