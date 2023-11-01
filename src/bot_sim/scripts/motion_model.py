#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Quaternion
import copy
import drifter_class as dft

import numpy as np

class MotionModel:
    def __init__(self,delta_t):
        self.sub = rospy.Subscriber("/cmd_vel",Twist,self.RecordVel,queue_size=10)
        self.vel = [0,0]
        self.sigma_lin = 0.5
        self.sigma_ang = 0.5
        self.delta_t = delta_t
        self.moving = False

    # def WrapToPosNegPi(self,theta):
        # if theta > 0:
            # theta = theta % (2*np.pi)
            # if theta > np.pi:
                # theta = theta - 2*np.pi
        # elif theta < 0:
            # theta = abs(theta) % (2*np.pi)
            # if theta > np.pi:
                # theta = -(theta - 2*np.pi)
        # return theta

    def WrapToPosNegPi(self,theta):
        while theta > np.pi:
            theta -= 2*np.pi
        while theta < -np.pi:
            theta += 2*np.pi
        return theta

    def getMoving(self):
        return self.moving

    def PredictMotion(self,robotPose):
        '''
        x_new = x + linear_velocity * delta_t * cos(theta)
        y_new = y + linear_velocity * delta_t * sin(theta)
        theta_new = theta + angular_velocity * delta_t
        '''
        pos = robotPose
        v = self.vel
        pos[0] = pos[0] + (v[0]+np.random.normal(0.0,self.sigma_lin)) * self.delta_t * np.cos(pos[2])
        pos[1] = pos[1] + (v[0]+np.random.normal(0.0,self.sigma_lin)) * self.delta_t * np.sin(pos[2])
        # pos[2] = pos[2] + (v[1]+np.random.normal(0.0,self.sigma_ang)) * self.delta_t
        pos[2] = pos[2] + v[1] * self.delta_t
        pos[2] = self.WrapToPosNegPi(pos[2])

        return pos

    def RecordVel(self,msg):
        self.vel[0] = msg.linear.x
        self.vel[1] = msg.angular.z
        if self.vel[0] != 0 or self.vel[1] != 0:
            self.moving = True
        else:
            self.moving = False

# below is test program
if __name__=="__main__":
    rospy.init_node("motion_model_p")
    delta_t = 0.1
    rate = rospy.Rate(1/delta_t)
    odom = rospy.wait_for_message("/odom",Odometry)
    pos = [0,0,0]
    pos[0] = odom.pose.pose.position.x
    pos[1] = odom.pose.pose.position.y
    qx = odom.pose.pose.orientation.x
    qy = odom.pose.pose.orientation.y
    qz = odom.pose.pose.orientation.z
    qw = odom.pose.pose.orientation.w
    pos[2] = euler_from_quaternion([qx,qy,qz,qw])[2]
    pub_pose = rospy.Publisher("/mm_pose",PoseStamped,queue_size=10)
    motion_model_pose = PoseStamped()
    frame = "odom"
    motion_model_pose.header.frame_id = frame
    motion_model_pose.pose.position.z = 0

    MotionModel = MotionModel(delta_t)
    # drifter = dft.Drifter(From=frame,To="odom")
    # drifter.ZeroDrift()
    q_pos = Quaternion()
    while not rospy.is_shutdown():
        pos = MotionModel.PredictMotion(pos)
        qx,qy,qz,qw = quaternion_from_euler(0,0,pos[2])
        motion_model_pose.pose.orientation.x = qx
        motion_model_pose.pose.orientation.y = qy
        motion_model_pose.pose.orientation.z = qz
        motion_model_pose.pose.orientation.w = qw
        motion_model_pose.pose.position.x = pos[0]
        motion_model_pose.pose.position.y = pos[1]

        pub_pose.publish(motion_model_pose)

        rate.sleep()
