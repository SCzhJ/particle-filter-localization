#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Quaternion
import drifter_class as dft

import numpy as np

class MotionModel:
    def __init__(self,delta_t):
        # get the changing velocity information
        self.sub = rospy.Subscriber("/cmd_vel",Twist,self.RecordVel,queue_size=10)
        
        # [linear velocity, angular velocity]
        self.vel = [0,0]

        self.sigma_lin = 0.05
        self.sigma_ang = 0.05

        self.delta_t = delta_t

    # /cmd_vel is in m/s, delta_t is the time duration in s
    # pos: position used for prediction: [x,y,theta]
    # given the velocity, time increment, and the current position, predict next time step's position
    def PredictMotion(self,pos):
        # build your own motion model, here's a reference
        '''
        x_new = x + linear_velocity * delta_t * cos(theta)
        y_new = y + linear_velocity * delta_t * sin(theta)
        theta_new = theta + angular_velocity * delta_t
        '''
        v = self.vel
        pos[0] = pos[0] + (v[0]+np.random.normal(0.0,self.sigma_lin)) * self.delta_t * np.cos(pos[2])
        pos[1] = pos[1] + (v[0]+np.random.normal(0.0,self.sigma_lin)) * self.delta_t * np.sin(pos[2])
        pos[2] = pos[2] + (v[1]+np.random.normal(0.0,self.sigma_ang)) * self.delta_t

        return pos

    def RecordVel(self,msg):
        self.vel[0] = msg.linear.x
        self.vel[1] = msg.angular.z

# below is test program
if __name__=="__main__":
    rospy.init_node("motion_model_p")
    delta_t = 0.01
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
