#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Quaternion
import copy
from sensor_msgs.msg import JointState

import numpy as np

class MotionModel:
    def __init__(self,delta_t):
        # self.robot_base_len = 0.2
        # self.wheel_radius = 0.0325
        # first_msg = rospy.wait_for_message("/joint_states",JointState)
        # self.wheel_right_angle = first_msg.position[0]
        # self.wheel_left_angle = first_msg.position[1]

        # self.wheel_right_prev_angle = self.wheel_right_angle
        # self.wheel_left_prev_angle = self.wheel_left_angle

        # self.sub_joint = rospy.Subscriber("/joint_states",JointState,self.RecordWheelRotation,queue_size=10)
        self.sub_cmd = rospy.Subscriber("/cmd_vel",Twist,self.RecordVel,queue_size=10)

        # self.v = 0
        # self.w = 0
        self.sigma_lin = 0.2
        self.sigma_ang = 0.2
        self.ang_coef = 0.7
        self.lin_coef = 0.92
        self.lin_vel = 0
        self.ang_vel = 0

        # self.not_move_threshold = 0.005

        self.delta_t = delta_t
        self.moving = False

    def PredictMotionCmd(self,particle):
        pos = dict()
        pos["x"] = particle["x"] + (self.lin_vel + 0) * self.delta_t * np.cos(particle["theta"])
        pos["y"] = particle["y"] + (self.lin_vel + 0) * self.delta_t * np.sin(particle["theta"])
        pos["theta"] = particle["theta"] + self.ang_coef * (self.ang_vel + 0) * self.delta_t
        pos["theta"] = self.WrapToPosNegPi(pos["theta"])
        pos["weight"] = particle["weight"]
        return pos

    def WrapToPosNegPi(self,theta):
        while theta > np.pi:
            theta -= 2*np.pi
        while theta < -np.pi:
            theta += 2*np.pi
        return theta

    def getMoving(self):
        return self.moving

    def RecordVel(self,msg):
        self.lin_vel = msg.linear.x
        self.ang_vel = msg.angular.z
        if msg.linear.x > 0 or msg.angular.z > 0:
            self.moving = True
        else:
            self.moving = False
    # def UpdateVelocity(self):
        # v_right = (self.wheel_right_angle-self.wheel_right_prev_angle)/self.delta_t * self.wheel_radius
        # self.wheel_right_prev_angle = self.wheel_right_angle
        # v_left = (self.wheel_left_angle-self.wheel_left_prev_angle)/self.delta_t * self.wheel_radius
        # self.wheel_left_prev_angle = self.wheel_left_angle
        # self.v = self.lin_coef * (v_right+v_left)/2
        # self.w = self.ang_coef * (v_right-v_left)/self.robot_base_len
        # if self.v < self.not_move_threshold and self.w < self.not_move_threshold:
            # self.moving = False
        # else:
            # self.moving = True

    # # particle as dict() of x, y, theta, and weight 
    # def PredictMotionJoint(self,particle,lin_sigma=0.01,ang_sigma=0.01):
        # '''
        # motion model applied:
        # x_new = x + linear_velocity * delta_t * cos(theta)
        # y_new = y + linear_velocity * delta_t * sin(theta)
        # theta_new = theta + angular_velocity * delta_t
        # '''
        # if self.moving:
            # lin_err = np.random.normal(0,lin_sigma)
            # ang_err = np.random.normal(0,ang_sigma)
        # else:
            # lin_err = 0
            # ang_err = 0
        # pos = dict()
        # pos["x"] = particle["x"] + (self.v + lin_err) * self.delta_t * np.cos(particle["theta"])
        # pos["y"] = particle["y"] + (self.v + ang_err) * self.delta_t * np.sin(particle["theta"])
        # pos["theta"] = particle["theta"] + (self.w + ang_err) * self.delta_t
        # pos["theta"] = self.WrapToPosNegPi(pos["theta"])
        # pos["weight"] = particle["weight"]
        # return pos

    # def RecordWheelRotation(self,msg):
        # self.wheel_right_angle = msg.position[0]
        # self.wheel_left_angle = msg.position[1]


# Test program
if __name__=="__main__":
    rospy.init_node("motion_model_p")
    delta_t = 0.02
    rate = rospy.Rate(1/delta_t)
    odom = rospy.wait_for_message("/odom",Odometry)
    pos = dict()
    pos["x"] = odom.pose.pose.position.x
    pos["y"] = odom.pose.pose.position.y
    qx = odom.pose.pose.orientation.x
    qy = odom.pose.pose.orientation.y
    qz = odom.pose.pose.orientation.z
    qw = odom.pose.pose.orientation.w
    pos["theta"] = euler_from_quaternion([qx,qy,qz,qw])[2]
    pos["weight"] = 0.001
    pub_pose = rospy.Publisher("/mm_pose",PoseStamped,queue_size=10)
    motion_model_pose = PoseStamped()
    frame = "odom"
    motion_model_pose.header.frame_id = frame
    motion_model_pose.pose.position.z = 0

    MotionModel = MotionModel(delta_t)
    q_pos = Quaternion()
    while not rospy.is_shutdown():
        pos = MotionModel.PredictMotionCmd(pos)
        qx,qy,qz,qw = quaternion_from_euler(0,0,pos["theta"])
        motion_model_pose.pose.orientation.x = qx
        motion_model_pose.pose.orientation.y = qy
        motion_model_pose.pose.orientation.z = qz
        motion_model_pose.pose.orientation.w = qw
        motion_model_pose.pose.position.x = pos["x"]
        motion_model_pose.pose.position.y = pos["y"]

        pub_pose.publish(motion_model_pose)

        rate.sleep()
