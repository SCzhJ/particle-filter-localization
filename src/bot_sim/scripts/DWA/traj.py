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

class Traj:
    """
    This class represents a trajectory of a robot.

    The trajectory is determined based on a linear velocity (`x_vel`), an angular velocity (`omega`), 
    a time step (`delta_t`), and the number of iterations to perform for each trajectory.

    Attributes:
        x_vel (float): The linear velocity of the robot.
        omega (float): The angular velocity of the robot.
        delta_t (float): The time step for the trajectory.
        iteration (int): The number of iterations to perform for each trajectory.
        R (float): The radius of the circle that the robot is moving along.
        rot (float): The rotation angle of the robot.
        rotation (np.ndarray): The rotation matrix of the robot.
        current_pose (np.ndarray): The current pose of the robot.
        poses (List[np.ndarray]): The list of poses recorded during the trajectory.

    Methods:
        get_traj(): Returns the list of poses recorded during the trajectory.
        straight_steer_once(robot_pose: np.ndarray): Steers the robot linearly once and returns the new pose.
        straight_steer(robot_pose: np.ndarray): Steers the robot linearly for a number of iterations and records the poses.
        steer_once_and_record_pose(robot_pose: np.ndarray, ICC: np.ndarray, A: np.ndarray): Steers the robot once and records the pose.
        steer_and_record_pose(robot_pose: np.ndarray): Steers the robot for a number of iterations and records the poses.
    """
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

    # returns: List[np.ndarray[3, 1]]
    def get_traj(self):
        """
        Return the list of poses recorded during the trajectory.

        Returns:
        List[np.ndarray]: The list of poses recorded during the trajectory.
        """
        return self.poses
    
    # robot_pose np.ndarray[3, 1]
    def straight_steer_once(self):
        """
        Steer the robot linearly once and return the new pose.

        Args:
        robot_pose (np.ndarray): The robot's current pose. Expected shape is (3, 1).

        Returns:
        np.ndarray: The robot's new pose after steering.
        """
        self.current_pose[0][0] += self.x_vel * np.cos(self.current_pose[2][0]) * self.delta_t
        self.current_pose[1][0] += self.x_vel * np.sin(self.current_pose[2][0]) * self.delta_t
        return copy.deepcopy(self.current_pose)
    
    def straight_steer(self, robot_pose):
        """
        Steer the robot linearly for a number of iterations and record the poses.

        Args:
        robot_pose (np.ndarray): The robot's current pose. Expected shape is (3, 1).

        Returns:
        np.ndarray: The robot's final pose after steering.
        """
        self.poses = []
        self.current_pose = copy.deepcopy(robot_pose)
        for _ in range(self.iteration):
            self.poses.append(self.straight_steer_once())

    def steer_once_and_record_pose(self, robot_pose, ICC, A):
        """
        Steer the robot once and record its pose.

        Args:
        robot_pose (np.ndarray): The robot's current pose. Expected shape is (3, 1).
        ICC (np.ndarray): The Instantaneous Center of Curvature.
        A (np.ndarray): The rotation matrix.

        Returns:
        np.ndarray: The robot's new pose after steering.
        """
        self.current_pose = A @ (robot_pose - ICC) + ICC + self.rotation
        return self.current_pose

    def steer_and_record_pose(self, robot_pose) -> None:
        """
        Steer the robot for a number of iterations and record the poses.

        Args:
        robot_pose (np.ndarray): The robot's current pose. Expected shape is (3, 1).
        """
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
    
    def get_trajectories(self):
        """
        Return the list of trajectories points, of dimension (n, 3, 1).
        """
        return self.wayposes
    
    def fill_trajectories(self, robot_pose) -> List:
        self.wayposes = []
        self.trajectories[0].straight_steer(robot_pose)
        self.wayposes.append(self.trajectories[0].get_traj())
        for i in range(1, len(self.traj_vels)):
            self.trajectories[i].steer_and_record_pose(robot_pose)
            self.wayposes.append(self.trajectories[i].get_traj())
        return self.wayposes

    def WrapToPosNegPi(self,theta):
        while theta > np.pi:
            theta -= 2*np.pi
        while theta < -np.pi:
            theta += 2*np.pi
        return theta

def initialize_traj_marker():
    # traj marker
    traj_marker = Marker()
    traj_marker.header.frame_id = "base_footprint"
    traj_marker.header.stamp = rospy.Time.now()
    traj_marker.ns = "traj_marker"
    traj_marker.id = 2
    traj_marker.type = Marker.LINE_LIST
    traj_marker.action = Marker.ADD
    traj_marker.scale.x = 0.01
    traj_marker.scale.y = 0.01
    traj_marker.color.r = 1.0
    traj_marker.color.g = 0.0
    traj_marker.color.b = 0.0
    traj_marker.color.a = 1.0
    traj_marker.pose.orientation.x = 0.0
    traj_marker.pose.orientation.y = 0.0
    traj_marker.pose.orientation.z = 0.0
    traj_marker.pose.orientation.w = 1.0
    traj_marker.lifetime = rospy.Duration()
    return traj_marker

def initialize_points_marker():
    # points marker
    points_marker = Marker()
    points_marker.header.frame_id = "map"
    points_marker.header.stamp = rospy.Time.now()
    points_marker.ns = "point_marker"
    points_marker.id = 3
    points_marker.type = Marker.POINTS
    points_marker.action = Marker.ADD
    points_marker.scale.x = 0.02
    points_marker.scale.y = 0.02
    points_marker.color.r = 0.0
    points_marker.color.g = 0.0
    points_marker.color.b = 1.0
    points_marker.color.a = 1.0
    points_marker.pose.orientation.x = 0.0
    points_marker.pose.orientation.y = 0.0
    points_marker.pose.orientation.z = 0.0
    points_marker.pose.orientation.w = 1.0
    points_marker.lifetime = rospy.Duration()
    # Add some points to the marker
    point = Point()
    point.x = 1.0
    point.y = 2.0
    point.z = 3.0
    points_marker.points.append(point)
    return points_marker


if __name__=="__main__":
    rospy.init_node("traj_test")
    dt = 0.05
    rate = rospy.Rate(1/dt)
    traj_pub = rospy.Publisher("traj_marker",Marker,queue_size=10)
    points_pub = rospy.Publisher("points_marker",Marker,queue_size=10)

    traj_marker = initialize_traj_marker()
    points_marker = initialize_points_marker()

    # Generate trajectory points
    traj_marker.points = []
    lin_vel = 0.2
    traj_vel = [[lin_vel, 0.0]]
    for w in range(1, 10):
        traj_vel.append([lin_vel,  w*0.05])
        traj_vel.append([lin_vel, -w*0.05])

    traj = TrajectoryRollout(traj_vel,delta_t=0.5,iteration=10)
    trajectories = traj.fill_trajectories(np.array([[0.0], [0.0], [0.0]]))
    
    for i in range(len(trajectories)):
        trajs = trajectories[i]
        prev_pose = trajs[0]
        for pose_i in range(1, len(trajs)):
            pose = trajs[pose_i]
            traj_marker.points.append(Point(prev_pose[0][0], prev_pose[1][0], 0.0))
            traj_marker.points.append(Point(pose[0][0], pose[1][0], 0.0))
            prev_pose = pose

    tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) # tf buffer length
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    ctime = 0
    while not rospy.is_shutdown():

        traj_pub.publish(traj_marker)
        try:
            trans = tf_buffer.lookup_transform('odom', 'base_link', rospy.Time.now(), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass
        if ctime >= 0.5:
            ctime = 0
            real_points = traj.get_real_world_points(trans.transform)
            new_points = []
            for poses in real_points:
                for pose in poses:
                    new_points.append(Point(pose[0][0], pose[1][0], 0.0))
            points_marker.points = new_points
            points_pub.publish(points_marker)
            
        rate.sleep()
        ctime+=dt



        



    
