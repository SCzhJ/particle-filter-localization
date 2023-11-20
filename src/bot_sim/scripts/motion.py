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
        self.sub_cmd = rospy.Subscriber("/cmd_vel",Twist,self.RecordVel,queue_size=10)

        self.sigma_lin = 0.2
        self.sigma_ang = 0.2
        self.ang_coef = 0.65
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


# Test program
if __name__=="__main__":
    rospy.init_node("motion_model_p")
    delta_t = 0.05
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
