#!/usr/bin/env python3

from typing import Any
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PointStamped
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from dwa_util import MapUtil
from traj import *
from test_util import *
from cost_function import *

import sys
import os
script_path = os.path.abspath(os.path.join(os.path.dirname(__file__),"..","RRT"))
print(script_path)
if script_path not in sys.path:
    sys.path.append(script_path)
from RRT_star import *

class RobotControl:
    def __init__(self, cost_map_path: str, candidate_vels: List[List[float]], dt: float, iteration_num: int, extension_iteration_num: int):
        self.cost_function = CostFunction(cost_map_path)
        self.cost_list = [0 for _ in range(len(candidate_vels))]
        # candidate_vels = [[v1, w1], [v2, w2], ...]
        self.candidate_vels = candidate_vels
        self.iteration_num = iteration_num
        self.traj_roll_out = TrajectoryRollout(candidate_vels, dt, iteration_num+extension_iteration_num)
        self.candidate_traj_points = []

        self.path_poses = []
        # Next point is the next point in the path that the robot should reach
        # When robot is sufficiently near the next point, 
        # it should be updated to the next nearest point
        self.next_point_index = 0
        self.tolerance = 0.3
    
    def set_path(self, path: List[Point]):
        '''
        path stored as list of points. The last term .z is used to store theta
        '''
        self.path_poses = copy.deepcopy(path)
        self.path_poses.append(copy.copy(path[-1]))
        self.next_point_index = 0
    
    def check_next_point_update(self, robot_pose: Point):
        '''
        check if the robot has reached the next point in the path
        '''
        next_point = self.path_poses[self.next_point_index]
        cart_dist = np.sqrt((robot_pose.x - next_point.x) ** 2 + (robot_pose.y - next_point.y) ** 2)
        while cart_dist < self.tolerance and self.next_point_index < len(self.path_poses)-1:
            self.next_point_index += 1
            next_point = self.path_poses[self.next_point_index]
            cart_dist = np.sqrt((robot_pose.x - next_point.x) ** 2 + (robot_pose.y - next_point.y) ** 2)
        if self.next_point_index == len(self.path_poses)-1:
            return True
        else:
            return False

    def calc_traj_points(self, transform):
        self.candidate_traj_points = self.traj_roll_out.get_real_world_points(transform)
    
    def get_candidate_traj_points(self):
        return self.candidate_traj_points
    
    def cost_of_all_traj(self):
        for i in range(len(self.candidate_traj_points)):
            self.cost_list[i] = self.cost_function.total_cost(self.candidate_traj_points[i], self.path_poses[self.next_point_index], self.iteration_num)
    
    def best_traj(self):
        '''
        Returns: int
        '''
        return self.cost_list.index(min(self.cost_list))

if __name__=="__main__":
    rospy.init_node("cost_function_p")

    dt = 0.01
    rate = rospy.Rate(1/dt)

    point_list_publisher = PointListPublisher()
    clicked_point_subscriber = ClickedPointSubscriber()
    odom_subscriber = OdomSubscriber()
    next_point_publisher = PointStampedPublisher('next_point_topic')
    cost_visualizer = CostVisualizer()
    cmd_publisher = CmdVelPublisher()
    pose_array_publisher = PoseArrayPublisher()

    folder_path = "/home/liyufei/RM/python-ROS-localization-and-planning/src/bot_sim/scripts/"
    rrt = RRTStar(folder_path + "RRT/CostMap/CostMapR05C005")

    # Generate trajectory points
    ang_vel_b = 0.3
    ang_vel_dif = 0.2
    ang_vel_spread = 7

    traj_vel = []

    lin_vel = 0.5
    lin_vel_decrement = 0.04

    traj_vel.append([0.05, 0.3])
    for w in range(1,ang_vel_spread):
        traj_vel.append([lin_vel-(ang_vel_spread-w)*lin_vel_decrement,  (ang_vel_spread-w)*ang_vel_dif + ang_vel_b])

    traj_vel.append([lin_vel, 0])
    for w in range(1,7):
        traj_vel.append([lin_vel-w*lin_vel_decrement, -w*ang_vel_dif - ang_vel_b])
    traj_vel.append([0.05, -0.3])

    print(traj_vel)

    update_interval = 0.02
    iteration_num = 10
    extension_iteration_num = 30
    robot_control = RobotControl(folder_path + "DWA/CostMap/CostMapA2d5B1d8", 
                                 traj_vel,
                                 update_interval,
                                 iteration_num,
                                 extension_iteration_num)
    robot_control.traj_roll_out.reduce_points(10)
    robot_control.traj_roll_out.fill_trajectories(np.array([[0],[0],[0]]))

    tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) # tf buffer length
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    path_exist = False
    path_publisher = PathPublisher()
    
    cum_time = 0
    while not rospy.is_shutdown():
        if clicked_point_subscriber.get_clicked():
            clicked_point_subscriber.set_clicked_false()
            path, mesg = rrt.rrt_plan_selection(odom_subscriber.robot_pose.position, 
                                                clicked_point_subscriber.get_clicked_point())
            if mesg == "found":
                path_exist = True
                robot_control.set_path(path)
                path_publisher.calc_path_from_point_list(path)
                path_publisher.publish_path()

                # publish path as pose array
                pose_array_publisher.pose_list_reset()
                for point in path:
                    pose_array_publisher.calc_point_add_list(point)
                pose_array_publisher.publish_pose_array()

            else:
                rospy.loginfo("path not found")
        if path_exist == True:
            if odom_subscriber.robot_pose != None:
                goal_reached = robot_control.check_next_point_update(odom_subscriber.robot_pose.position)
                next_point = robot_control.path_poses[robot_control.next_point_index]
                next_point.z = 0.0
                next_point_publisher.publish_point(next_point)
                if goal_reached == True:
                    rospy.loginfo("goal reached!")
                    path_exist = False
                    cmd_publisher.publish_cmd_vel(0,0)
        if cum_time >= update_interval * iteration_num and path_exist == True:
            cum_time = 0

            # calculate trajectory points and publish
            try:
                trans = tf_buffer.lookup_transform('odom', 'base_link', rospy.Time.now(), rospy.Duration(1.0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                pass
            robot_control.calc_traj_points(trans.transform)
            real_points = robot_control.get_candidate_traj_points()
            new_points = []
            for poses in real_points:
                for pose in poses:
                    new_points.append(Point(pose[0][0], pose[1][0], 0.0))
            point_list_publisher.publish_point_list(new_points)

            # calculate trajectory cost
            robot_control.cost_of_all_traj()
            best_traj_i = robot_control.best_traj()
            # cost_visualizer.visualize(robot_control.cost_list)
            # rospy.loginfo(best_traj_i)
            cmd_publisher.publish_cmd_vel(robot_control.candidate_vels[best_traj_i][0],
                                          robot_control.candidate_vels[best_traj_i][1])
        cum_time += dt
        rate.sleep()
    cmd_publisher.publish_cmd_vel(0,0)
