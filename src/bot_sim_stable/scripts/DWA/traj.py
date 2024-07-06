#!/usr/bin/env python3

import rospy
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from geometry_msgs.msg import Transform
from tf.transformations import euler_from_quaternion
from typing import List
import copy
import tf2_ros

'''
All robot pose represented by three elements are:
    x, y, theta
'''

class Traj:
    def __init__(self, x_vel: float = 0.3, omega: float = 0.3, delta_t: float = 0.05, iteration: int = 10):
        """
        Initialize the Traj object.
        """
        self.x_vel = x_vel
        self.omega = 0.001 if omega == 0.0 else omega
        self.poses = []
        self.delta_t = delta_t
        self.iteration = iteration
        self.R = self.x_vel / self.omega
        self.rot = self.omega * self.delta_t
        self.rotation = np.array([[0], [0], [self.rot]])
        self.current_pose = None
    
    def point_reduction(self, num_of_points_retained: int = 10):
        """
        Reduce the number of points in the trajectory to a certain number.
        """
        if len(self.poses) <= num_of_points_retained:
            return
        else:
            self.poses = self.poses[::len(self.poses)//num_of_points_retained]

    # returns: List[np.ndarray[3, 1]]
    def get_traj(self):
        return self.poses
    
    # robot_pose np.ndarray[3, 1]
    def straight_steer_once(self):
        self.current_pose[0][0] += self.x_vel * np.cos(self.current_pose[2][0]) * self.delta_t
        self.current_pose[1][0] += self.x_vel * np.sin(self.current_pose[2][0]) * self.delta_t
        return copy.deepcopy(self.current_pose)
    
    def straight_steer(self, robot_pose):
        self.poses = []
        self.current_pose = copy.deepcopy(robot_pose)
        for _ in range(self.iteration):
            self.poses.append(self.straight_steer_once())

    def steer_once_and_record_pose(self, robot_pose, ICC, A):
        self.current_pose = A @ (robot_pose - ICC) + ICC + self.rotation
        return self.current_pose

    def steer_and_record_pose(self, robot_pose) -> None:
        ICC = np.array([[robot_pose[0][0] - self.R * np.sin(robot_pose[2][0])],
                        [robot_pose[1][0] + self.R * np.cos(robot_pose[2][0])],
                        [0.0]])
        A = np.array([[np.cos(self.rot), -np.sin(self.rot), 0.0],
                      [np.sin(self.rot), np.cos(self.rot), 0.0],
                      [0.0, 0.0, 1.0]])
        self.poses = []
        self.current_pose = copy.deepcopy(robot_pose)
        for _ in range(self.iteration):
            self.poses.append(self.steer_once_and_record_pose(self.current_pose, ICC, A))

class TrajectoryRollout:
    def __init__(self, traj_vels: List[List[float]], delta_t: float = 0.05, iteration: int = 10):
        """
        Initialize the TrajectoryRollout object.
        """
        self.traj_vels = traj_vels
        self.delta_t = delta_t
        self.iteration = iteration
        self.trajectories = [Traj(x_vel=self.traj_vels[0][0], omega=self.traj_vels[0][1], delta_t=self.delta_t, iteration=self.iteration)]
        for i in range(1, len(self.traj_vels)):
            self.trajectories.append(Traj(x_vel=self.traj_vels[i][0], omega=self.traj_vels[i][1], delta_t=self.delta_t, iteration=self.iteration))
        self.wayposes = []
    
    def get_real_world_points(self, transform):
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
        point_trajs = []
        for poses in self.wayposes:
            real_points = []
            for pose in poses:
                homo_point[0][0] = pose[0][0]
                homo_point[1][0] = pose[1][0]
                new_point = trans_mat @ homo_point
                new_point [2][0] = self.WrapToPosNegPi(pose[2][0] + yaw)
                real_points.append(new_point)
            point_trajs.append(real_points)
        return point_trajs
    
    # Returns List[np.ndarray(3,1)]
    def get_trajectories(self):
        """
        Return the list of trajectories points, of dimension (n, 3, 1).
        """
        return self.wayposes
    
    def fill_trajectories(self, robot_pose) -> List:
        self.wayposes = []
        for i in range(len(self.traj_vels)):
            if self.traj_vels[i][0] == 0:
                self.trajectories[i].straight_steer(robot_pose)
                self.wayposes.append(self.trajectories[i].get_traj())
            else:
                self.trajectories[i].steer_and_record_pose(robot_pose)
                self.wayposes.append(self.trajectories[i].get_traj())
        return self.wayposes
    
    def reduce_points(self, num_of_points_retained: int = 10):
        for i in range(len(self.trajectories)):
            self.trajectories[i].point_reduction(num_of_points_retained)

    def WrapToPosNegPi(self,theta):
        while theta > np.pi:
            theta -= 2*np.pi
        while theta < -np.pi:
            theta += 2*np.pi
        return theta
