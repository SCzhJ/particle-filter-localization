#!/usr/bin/env python3
import rospy
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class CurveTraj:
    def __init__(self, x_vel=0.3, omega=0.3, delta_t=0.05, iteration=10):
        self.x_vel = x_vel
        self.omega = None
        self.OmegaNoneZeroDetection(omega)
        self.poses = []
        self.delta_t = delta_t
        self.iteration = iteration
        # The following kinametics are calculated
        # using the 3rd lecture of introduction to
        # mobile robotics
        self.R = self.x_vel/self.omega
        self.rot = self.omega*self.delta_t
        self.rotation = np.array([[0],[0],[self.rot]])
        self.current_pose = None
    
    def OmegaNoneZeroDetection(self, omega):
        if omega == 0.0:
            rospy.logerr("Used Zero Omega in Curve Trajectory! Division of Zero might occur!")
            self.omega = 0.001
        else:
            self.omega = omega
    
    def getTraj(self):
        return self.poses

    # robotPose is 3*1 numpy array [[x],[y],[theta]]
    # ICC is 3*1 numpy array [[ICCx],[ICCy],[0]]
    # A is 3*3 rotation matrix applied to the robot
    def StearOnceAndRecordPose(self,robotPose,ICC,A):
        self.current_pose = A @ (robotPose - ICC) + ICC + self.rotation
        return self.current_pose

    # robotPose is 3*1 numpy array [[x],[y],[theta]]
    def StearAndRecordPose(self,robotPose):
        ICC = np.array([[robotPose[0][0]-self.R*np.sin(robotPose[2][0])],
                        [robotPose[1][0]+self.R*np.cos(robotPose[2][0])],
                        [0.0                                           ]])
        A   = np.array([[np.cos(self.rot),-np.sin(self.rot),0.0],
                        [np.sin(self.rot), np.cos(self.rot),0.0],
                        [0.0             ,0.0             ,1.0]])
        self.poses = []
        self.current_pose = robotPose
        for i in range(self.iteration):
            self.poses.append(self.StearOnceAndRecordPose(self.current_pose,ICC,A))

if __name__=="__main__":
    rospy.init_node("traj_test")
    rate = rospy.Rate(10)
    traj_pub = rospy.Publisher("traj_marker",Marker,queue_size=10)
    # traj marker
    traj_marker = Marker()
    traj_marker.header.frame_id = "map"
    traj_marker.header.stamp = rospy.Time.now()
    traj_marker.ns = "traj_marker"
    traj_marker.id = 2
    traj_marker.type = Marker.LINE_STRIP
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

    traj = CurveTraj(x_vel=1, omega=5, delta_t=0.01, iteration=20)
    robotPose = np.array([[0],[0],[0]])
    traj.StearAndRecordPose(robotPose)
    wayposes = traj.getTraj()

    print(wayposes)
    waypoints = []
    for pose in wayposes:
        point = Point()
        point.x = pose[0][0]
        point.y = pose[1][0]
        point.z = 0.1
        waypoints.append(point)
    
    traj_marker.points = waypoints
    print(waypoints)
    while not rospy.is_shutdown():
        traj_pub.publish(traj_marker)
        rate.sleep()



        



    
