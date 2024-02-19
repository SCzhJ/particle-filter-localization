#!/usr/bin/env python3

import rospy
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from test_util import PointListPublisher
from geometry_msgs.msg import Transform
from tf.transformations import euler_from_quaternion
from typing import List
import copy
import pickle
import tf2_ros

'''
All robot pose represented by three elements are:
    x, y, theta
'''
class TrajectoryMode:
    def __init__(self):
        self.velocities = []
        self.velocity_set = False
    
    def getVelocities(self):
        return self.velocities
    
    def getVelocitySet(self):
        return self.velocity_set
    
    def clearVelocitySet(self):
        self.velocity_set = False
    
    def ShiftTrajectory(self, linear_vel: float, num_of_traj: int):
        """
        Trajectories scattering around the robot, no angular velocity
        """
        self.velocities = []
        theta = 0
        for _ in range(num_of_traj):
            self.velocities.append([linear_vel * np.cos(theta), linear_vel * np.sin(theta), 0])
            theta += np.pi * 2 / num_of_traj
        self.velocity_set = True
    
    def DifferentialDriveTrajectory(self, x_vel: float, omega_increment: float, num_of_traj_one_side: int):
        """
        Trajectories of differential drive robot, without y velocity
        """
        self.velocities = [[x_vel, 0, 0]]
        for i in range(num_of_traj_one_side):
            self.velocities.append([x_vel, 0, omega_increment * (i+1)])
            self.velocities.append([x_vel, 0, -omega_increment * (i+1)])
        self.velocity_set = True

class TrajectoryGenerator:
    def __init__(self, delta_t: float, record_iteration: int, iteration: int):
        """
        Given the velocity and angular velocity, generate the trajectory of the robot.

        x_vel: float, velocity in x direction
        y_vel: float, velocity in y direction
        omega: float, angular velocity
        delta_t: float, time step
        record_iteration: int, record the trajectory point every record_iteration time steps
        iteration: int, number of points being recorded in single trajectory
        robot_pose: ndarray[3,1], robot pose [[x],[y],[theta]]
        """
        self.x_vel = None
        self.y_vel = None
        self.omega = None
        self.delta_t = delta_t
        self.record_iteration = record_iteration
        self.iteration = iteration
        self.robot_pose = np.zeros((3, 1))
        self.traj = []
        self.trajectories = []
    
    def getTrajectories(self):
        return self.trajectories
    
    def SetVel(self, x_vel: float, y_vel: float, omega: float):
        self.x_vel = x_vel
        self.y_vel = y_vel
        self.omega = omega

    def GenTrajPoint(self):
        """
        Generate single trajectory point
        robot_pose: ndarray[3,1]
        """
        for _ in range(self.record_iteration):
            self.robot_pose[0][0] += self.x_vel * np.cos(self.robot_pose[2][0]) * self.delta_t
            self.robot_pose[1][0] += self.x_vel * np.sin(self.robot_pose[2][0]) * self.delta_t
            self.robot_pose[2][0] += self.omega * self.delta_t
        self.traj.append(copy.deepcopy(self.robot_pose))

    def GenTraj(self):
        """
        Generate Trajectory
        """
        self.traj = []
        self.robot_pose = np.zeros((3, 1))
        for _ in range(self.iteration):
            self.GenTrajPoint()
    
    def GenTrajectories(self, vel_list: List):
        """
        Generate Trajectories
        """
        self.trajectories = []
        for vel in vel_list:
            self.SetVel(vel[0], vel[1], vel[2])
            self.GenTraj()
            self.trajectories.append(copy.deepcopy(self.traj))

class TrajectoryManagement:
    def __init__(self, traj_gen: TrajectoryGenerator, traj_mode: TrajectoryMode):
        """
        traj_gen:  initialized TrajectoryGenerator
        traj_mode: initialized TrajectoryMode
        """
        self.traj_gen  = traj_gen
        self.traj_mode = traj_mode
        self.trajectories = []
        self.PointListPublisher = PointListPublisher(marker_id=17, topic_name='traj_points')
    
    def GenTrajectories(self):
        """
        Return the list of trajectories points, of dimension (n, 3, 1).
        """
        if self.traj_mode.getVelocitySet() == False:
            rospy.logerr("Velocity of TrajectoryMode not set")
        else:
            self.traj_gen.GenTrajectories(self.traj_mode.getVelocities())
            self.trajectories = self.traj_gen.getTrajectories()
            self.traj_mode.clearVelocitySet()
    
    def VisualizeTrajectories(self):
        """
        Visualize the trajectories
        """
        point_list = []
        for traj in self.trajectories:
            for array in traj:
                point_list.append(Point(array[0][0], array[1][0], 0))
        self.PointListPublisher.publish_point_list(point_list)
    
    def StoreTrajectories(self, path: str):
        """
        Store the trajectories
        """
        pickle.dump(self.trajectories, open(path, "wb"))
    
    def ReadTrajectories(self, path: str):
        """
        Read the trajectories
        """
        self.trajectories = pickle.load(open(path, "rb"))

"""
To be modified: New code should read file storing trajectory points
"""
class TrajectoryRollout:
    def __init__(self, traj_vels: List[List[float]], delta_t: float = 0.05, iteration: int = 10):
        """
        Initialize the TrajectoryRollout object.
        """
        self.traj_vels = traj_vels
        self.delta_t = delta_t
        self.iteration = iteration

        self.robot_frame_traj = []
        self.world_frame_traj = []
    
    def ReadTrajectories(self, path: str):
        """
        Read the trajectories
        """
        self.robot_frame_traj = pickle.load(open(path, "rb"))
    
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
        self.world_frame_traj = []
        for poses in self.robot_frame_traj:
            real_points = []
            for pose in poses:
                homo_point[0][0] = pose[0][0]
                homo_point[1][0] = pose[1][0]
                new_point = trans_mat @ homo_point
                new_point [2][0] = self.WrapToPosNegPi(pose[2][0] + yaw)
                real_points.append(new_point)
            self.world_frame_traj.append(real_points)
    
    # Returns List[np.ndarray(3,1)]
    def get_robot_frame_trajectories(self):
        """
        Return the list of trajectories points, of dimension (n, 3, 1).
        """
        return self.robot_frame_traj

    # Returns List[np.ndarray(3,1)]
    def get_world_frame_trajectories(self):
        """
        Return the list of trajectories points, of dimension (n, 3, 1).
        """
        return self.world_frame_traj

    def WrapToPosNegPi(self,theta):
        while theta > np.pi:
            theta -= 2*np.pi
        while theta < -np.pi:
            theta += 2*np.pi
        return theta



# Test Program
if __name__ == "__main__":
    rospy.init_node("trajectory_node")
    rate = rospy.Rate(10)
    traj_gen = TrajectoryGenerator(delta_t=0.1, record_iteration=5, iteration=5)
    traj_mode = TrajectoryMode()
    traj_mode.DifferentialDriveTrajectory(x_vel=1, omega_increment=0.1, num_of_traj_one_side=2)
    print(traj_mode.getVelocities())
    traj_manage = TrajectoryManagement(traj_gen, traj_mode)

    traj_manage.GenTrajectories()
    print(traj_manage.traj_gen.getTrajectories())
    while not rospy.is_shutdown():
        traj_manage.VisualizeTrajectories()
        rate.sleep()