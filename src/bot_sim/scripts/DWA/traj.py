#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Point
from geometry_msgs.msg import Transform
from tf.transformations import euler_from_quaternion
from typing import List
import copy
import pickle
import tf2_ros
from test_util_2 import PointListPublisher, InitialposeSubscriber

def WrapToPosNegPi(theta):
    while theta > np.pi:
        theta -= 2*np.pi
    while theta < -np.pi:
        theta += 2*np.pi
    return theta

class trajObject:
    def __init__(self, x_vel, y_vel, omega, cost_type="calculated", proposed_cost=0):
        # linear velocity in x, y directions and angular velocity
        self.x_vel = x_vel
        self.y_vel = y_vel
        self.omega = omega
        # trajectory poses of the robot frame, each pose in list is np.array([[x], [y], [theta]])
        self.traj_poses = []
        # world frame trajectory poses, each pose in list is np.array([[x], [y], [theta]])
        self.world_frame_traj_poses = []
        # cost types: 
        # "calculated": the proposed cost is not used, but the cost is calculated by the trajectory
        # "constant":   the proposed cost is used, not calculated from trajectory
        self.cost_type = cost_type
        self.proposed_cost = proposed_cost

    def SetTrajPoses(self, traj_poses):
        self.traj_poses = traj_poses

    def CalcWorldFramePoses(self, transform):
        '''
        Returns: List[List[np.ndarray[3, 1]]]
        '''
        _, _, yaw = euler_from_quaternion([transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w])
        dx        = transform.translation.x
        dy        = transform.translation.y
        homo_point= np.array([[0.0], [0.0], [1.0]])
        trans_mat = np.array([[np.cos(yaw), -np.sin(yaw), dx],
                              [np.sin(yaw),  np.cos(yaw), dy],
                              [          0,            0,  1]])
        self.world_frame_traj_poses = []
        for pose in self.traj_poses:
            homo_point[0][0] = pose[0][0]
            homo_point[1][0] = pose[1][0]
            new_point = trans_mat @ homo_point
            new_point [2][0] = WrapToPosNegPi(pose[2][0] + yaw)
            self.world_frame_traj_poses.append(new_point)

'''
All robot pose represented by three elements are:
    x, y, theta
'''
class TrajectoryMode:
    @staticmethod
    def ShiftTrajectories(linear_vel: float, num_of_traj: int) -> List[trajObject]:
        """
        Set velocity for trajectories scattering around the robot, no angular velocity
        """
        trajectories = []
        theta = 0
        for i in range(num_of_traj):
            trajectories.append(trajObject(linear_vel * np.cos(theta), linear_vel * np.sin(theta), 0))
            theta += np.pi * 2 / num_of_traj
        return trajectories
    
    @staticmethod
    def DifferentialDriveTrajectory(x_vel: float, omega_increment: float, num_of_traj_one_side: int) -> List[trajObject]:
        """
        Set velocity for trajectories of differential drive robot, no y velocity
        """
        trajectories = []
        for i in range(num_of_traj_one_side):
            trajectories.append(trajObject(x_vel, 0, omega_increment * (num_of_traj_one_side - i)))
        trajectories.append(trajObject(x_vel, 0, 0))
        for i in range(num_of_traj_one_side):
            trajectories.append(trajObject(x_vel, 0, -omega_increment * (i+1)))
        return trajectories
    
    @staticmethod
    def GenTrajectory(traj: trajObject, dt: float = 0.1, record_every_iter: int = 10, iteration: int = 10):
        """
        Generate Trajectories from given velocity
        """
        robot_pose = np.zeros((3, 1))
        delta_t = dt / record_every_iter
        for j in range(iteration):
            for k in range(record_every_iter):
                robot_pose[0][0] += traj.x_vel * delta_t * np.cos(robot_pose[2][0]) + traj.y_vel * delta_t * (-np.sin(robot_pose[2][0]))
                robot_pose[1][0] += traj.y_vel * delta_t * np.cos(robot_pose[2][0]) + traj.x_vel * delta_t *   np.sin(robot_pose[2][0])
                robot_pose[2][0] += traj.omega * delta_t
            traj.traj_poses.append(copy.deepcopy(robot_pose))
    

class TrajectoryManagement:
    def __init__(self):
        self.PointListPublisher = PointListPublisher(marker_id=17, topic_name='traj_points')

    
    def VisualizeTrajectories(self, trajectories: List[trajObject]):
        point_list = []
        for traj in trajectories:
            for array in traj.traj_poses:
                point_list.append(Point(array[0][0], array[1][0], 0))
            for array in traj.world_frame_traj_poses:
                point_list.append(Point(array[0][0], array[1][0], 0))
        self.PointListPublisher.publish_point_list(point_list)
    
    def StoreTrajectories(self, path: str, trajectories: List[trajObject]):
        pickle.dump(trajectories, open(path, "wb"))
    
    def ReadTrajectories(self, path: str) -> List[trajObject]:
        return pickle.load(open(path, "rb"))







# Test Program
if __name__ == "__main__":
    rospy.init_node("trajectory_node")
    rate = rospy.Rate(10)

    trajectories = TrajectoryMode.DifferentialDriveTrajectory(x_vel=1, omega_increment=0.3, num_of_traj_one_side=4)
    # trajectories = traj_mode.ShiftTrajectories(linear_vel=1, num_of_traj=10)

    for traj in trajectories:
        TrajectoryMode.GenTrajectory(traj, dt=0.1, record_every_iter=10, iteration=10)
    for traj in trajectories:
        traj.traj_poses = traj.traj_poses[5:]

    traj_manage = TrajectoryManagement()
    initpose_sub = InitialposeSubscriber()

    while not rospy.is_shutdown():
        if initpose_sub.initialpose is not None:
            trans = initpose_sub.get_pose_transform()
            for traj in trajectories:
                traj.CalcWorldFramePoses(trans)
        traj_manage.VisualizeTrajectories(trajectories)
        rate.sleep()