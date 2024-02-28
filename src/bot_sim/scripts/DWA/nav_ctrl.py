#!/usr/bin/env python3

from typing import Any
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import Point
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
import tf2_ros

from bot_sim.srv import RRTStar

import sys
import os
script_path = os.path.abspath(os.path.join(os.path.dirname(__file__),"..","RRT"))
if script_path not in sys.path:
    sys.path.append(script_path)
rospy.loginfo("Added path: %s", script_path)
from RRT_star import *

script_path = os.path.abspath(os.path.join(os.path.dirname(__file__),"..","DWA"))
if script_path not in sys.path:
    sys.path.append(script_path)
rospy.loginfo("Added path: %s", script_path)
from dwa_util import MapUtil
from traj import *
from test_util import *
from cost_function import *


class NavCtrl:
    def __init__(self, trajectories: List[trajObject], cost_map_path: str, dyn_map_name: str,
                 dt: float, traj_cut_percentage: float = 0.5, iter_percentage: float = 0.5,
                 pathfollowext_percentage: float = 0.8, cmd_vel_topic: str = "/cmd_vel",
                 within_point: float = 1.5, within_goal: float = 0.5, cost_method: str = "omni"):

        self.trajectories = trajectories
        # Cut trajectory points to only keep remote points
        self.traj_cut_index = int(traj_cut_percentage * len(trajectories[0].traj_poses))
        for traj in trajectories:
            traj.traj_poses = traj.traj_poses[self.traj_cut_index:]
        # These value used for cost function
        # path_iter is used for bearing cost and turn cost
        # pathfollowext_iter is used for path following cost
        # obstacbleext_iter is used for obstacle avoidance cost
        path_iter = int(iter_percentage * len(trajectories[0].traj_poses))
        pathfollowext_iter = int(pathfollowext_percentage * len(trajectories[0].traj_poses))
        obsext_iter = len(trajectories[0].traj_poses) - path_iter - pathfollowext_iter

        self.cost_func = CostFunction(cost_map_path, dyn_map_name,path_iter, pathfollowext_iter, obsext_iter)
        if cost_method == "omni":
            self.cost = self.cost_func.total_cost_omni
        else:
            self.cost = self.cost_func.total_cost

        # transform as robot location
        self.loc_trans = None
        self.robot_frame = rospy.get_param("~robot_frame")
        self.world_frame = rospy.get_param("~world_frame")
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) # tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Path Control
        self.path = []
        self.path_len = -1
        
        # Update Next Point Condition
        self.within_point = within_point
        self.next_point_index = 0

        # Goal Reach Condition
        self.within_goal = within_goal

        # other
        self.cmd_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        self.dt = dt
    
    def call_rrt_star(self, curr_x: float, curr_y: float, goal_x: float, goal_y: float):
        rospy.wait_for_service('rrt_star')
        try:
            rrt_star = rospy.ServiceProxy('rrt_star', RRTStar)
            resp = rrt_star(curr_x, curr_y, goal_x, goal_y)
            if resp.success == 1:
                self.path_len = len(resp.path)
                self.path = resp.path
                return resp.success
            else:
                return resp.success
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return -1
    
    def update_location(self):
        try:
            self.loc_tran = self.tf_buffer.lookup_transform(self.world_frame, self.robot_frame,
                                                    rospy.Time.now(), rospy.Duration(1.0))#写成两个param
            for traj in self.trajectories:
                traj.CalcWorldFramePoses(self.loc_tran.transform)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass
    
    def get_next_point(self, curr_x: float, curr_y: float, goal_x: float, goal_y: float):
        if self.next_point_index < self.path_len:
            next_point = self.path[self.next_point_index]
            if abs(next_point.x - curr_x) < self.within_point \
                and abs(next_point.y - curr_y) < self.within_point:
                self.next_point_index += 1
                return self.get_next_point(curr_x, curr_y, goal_x, goal_y)
            else:
                return next_point
        else:
            return Point(goal_x, goal_y, 0)
    
    def check_goal_reached(self, curr_x: float, curr_y: float, goal_x: float, goal_y: float):
        if abs(goal_x - curr_x) < self.within_goal and abs(goal_y - curr_y) < self.within_goal:
            return True
        else:
            return False
    
    def check_best_traj_vel(self, next_point: Point):
        min_cost = float('inf')
        best_traj = None
        scaler = np.sqrt((next_point.x - self.loc_tran.transform.translation.x)**2 + \
                         (next_point.y - self.loc_tran.transform.translation.y)**2)
        for traj in self.trajectories:
            cost = self.cost(traj.world_frame_traj_poses, traj.traj_poses, next_point, scaler)
            if cost < min_cost:
                min_cost = cost
                best_traj = traj
        return best_traj.x_vel, best_traj.y_vel, best_traj.omega
    
    def nav_to_goal(self, goal_x: float, goal_y: float) -> int:
        self.next_point_index = 0
        self.update_location()
        if self.loc_tran == None:
            rospy.logerr("No location found")
            return -1
        path_get = self.call_rrt_star(self.loc_tran.transform.translation.x, 
                                      self.loc_tran.transform.translation.y, goal_x, goal_y)
        if path_get != 1:
            rospy.logerr("path not found")
            return -1
        else:
            # path publisher
            path_publisher = PathPublisher()
            path_publisher.calc_path_from_point_list(self.path)
            # publish point
            next_point_publisher = PointStampedPublisher('next_point_topic')
            point_list_publisher = PointListPublisher(frame_id="map")

            rate = rospy.Rate(1/self.dt)
            vel = Twist()
            while not self.check_goal_reached(self.loc_tran.transform.translation.x, 
                                              self.loc_tran.transform.translation.y, goal_x, goal_y) and \
                                                not rospy.is_shutdown():
                next_point = self.get_next_point(self.loc_tran.transform.translation.x, 
                                                 self.loc_tran.transform.translation.y, 
                                                 goal_x, goal_y)
                pub_point = next_point
                pub_point.z = 0
                next_point_publisher.publish_point(pub_point)

                x_vel, y_vel, omega = self.check_best_traj_vel(next_point)

                vel.linear.x = x_vel
                vel.linear.y = y_vel
                vel.angular.z = omega
                self.cmd_publisher.publish(vel)

                traj_point_list = []
                for traj in self.trajectories:
                    for point in traj.world_frame_traj_poses:
                        traj_point_list.append(Point(point[0][0], point[1][0], 0.1))
                point_list_publisher.publish_point_list(traj_point_list)

                path_publisher.publish_path()

                self.update_location()
                rate.sleep()

            rospy.loginfo("goal reached")
            vel.linear.x = 0
            vel.linear.y = 0
            vel.angular.z = 0
            self.cmd_publisher.publish(vel)
            return 1
        
if __name__=="__main__":
    rospy.init_node('nav_ctrl')

    dt = 0.2
    # trajectories = TrajectoryMode.DifferentialDriveTrajectory(x_vel=0.5, omega_increment=0.1, num_of_traj_one_side=7) + \
    #     TrajectoryMode.DifferentialDriveTrajectory(x_vel=-0.5, omega_increment=0.1, num_of_traj_one_side=7)
    trajectories = TrajectoryMode.ShiftTrajectories(linear_vel=1.0, num_of_traj=15)

    for traj in trajectories:
        TrajectoryMode.GenTrajectory(traj, dt, record_every_iter=3, iteration=5)
    
    cost_map_path = rospy.get_param("~cost_map_path")
    dyn_map_name = rospy.get_param("~dyn_map_name")    
    NavCtrl = NavCtrl(trajectories, cost_map_path, dyn_map_name, dt, traj_cut_percentage=0.2)

    clicked_point_subscriber = ClickedPointSubscriber()

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        if clicked_point_subscriber.clicked_point is not None:
            NavCtrl.nav_to_goal(clicked_point_subscriber.clicked_point.x, clicked_point_subscriber.clicked_point.y)
            clicked_point_subscriber.clicked_point = None
        rate.sleep()
    

