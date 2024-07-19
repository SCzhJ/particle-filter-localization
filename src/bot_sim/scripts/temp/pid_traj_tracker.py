#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
import math

class PIController:
	def __init__(self, kp, ki):
		self.kp = kp
		self.ki = ki
		self.integral = 0.0

	def compute(self, error, dt):
		self.integral += error * dt
		return self.kp * error + self.ki * self.integral

class TrajectoryTracker:
	def __init__(self):
		rospy.init_node('trajectory_tracker')
		
		self.goal_point = [0, 1]  # Example goal point
		self.origin_point = [0, 0]
		self.current_position = Point()
		self.orthoganol_vec = [self.goal_point[0] - self.origin_point[0], self.goal_point[1] - self.origin_point[1]]
        # rotate the vector by 90 degrees
		self.orthoganol_vec = [-self.orthoganol_vec[1], self.orthoganol_vec[0]]
		
		self.base_linear_velocity = 0.5  # Base linear velocity
		self.pi_controller = PIController(kp=1.0, ki=0.1)
		
		self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		rospy.Subscriber('/odom', Odometry, self.odom_callback)
		
		self.rate = rospy.Rate(10)  # 10 Hz


	def odom_callback(self, msg):
		self.current_position = msg.pose.pose.position



	def compute_control(self):


if __name__ == "__main__":
    tt = TrajectoryTracker()
    while not rospy.is_shutdown():
        tt.compute_control()
        tt.rate.sleep()