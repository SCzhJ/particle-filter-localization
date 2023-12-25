#!/usr/bin/env python3

from typing import Any
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from dwa_util import MapUtil
from traj import *

import sys
import os
script_path = os.path.abspath(os.path.join(os.path.dirname(__file__),"..","RRT"))
print(script_path)
if script_path not in sys.path:
    sys.path.append(script_path)
from RRT_star import *

class CostFunction:
    '''
    Implementation of Dynamic Window Approach's cost function
    '''
    def __init__(self, cost_map_path: str, candidate_vels: List[List[float]], dt: float = 0.05, iteration: int = 10):
        self.map_util = MapUtil()
        self.map_util.load_cost_map(cost_map_path)
        # candidate_vels = [[v1, w1], [v2, w2], ...]
        self.candidate_vels = candidate_vels
        self.traj_roll_out = TrajectoryRollout(candidate_vels, dt, iteration)
        self.traj_roll_out.fill_trajectories(np.array([[0],[0],[0]]))
        self.candidate_traj_points = []

        # cost function
        self.cost_list = []

        # path stored as list of points. The last term .z is used to store theta
        self.path_poses = []
        # Next point is the next point in the path that the robot should reach
        # When robot is sufficiently near the next point, 
        # it should be updated to the next nearest point
        self.next_point_index = 0
        self.tolerance = 0.3

        # path follow cost = dist_cost + bearing_cost
        # dist_cost = k * cartesian-distance ^ m
        self.dist_cost_k = 1.6
        self.dist_cost_m = 1
        # bearing_cost = k * |bearing - next_point_bearing| ^ m
        self.dist_cost_k = 0.5

        # collision cost = k * (N - i) ^ m + b
        # where N is the total number of points in the trajectory,
        # and i ~ [0, N-1] is the first index to collide
        self.coll_k = 1.2
        self.coll_m = 0.8
        self.coll_b = 0.5

        # turn penalty = k * |theta| ^ m
        self.turn_k = 0.5
        self.turn_m = 0.4
    
    def set_path(self, path: List[Point]):
        '''
        path stored as list of points. The last term .z is used to store theta
        '''
        self.path_poses = path
        self.next_point_index = 0
    
    def check_next_point_update(self, robot_pose: Point, tolerance: float = 0.1):
        '''
        check if the robot has reached the next point in the path
        '''
        next_point = self.path_poses[self.next_point_index]
        cart_dist = np.sqrt((robot_pose.x - next_point.x) ** 2 + (robot_pose.y - next_point.y) ** 2)
        while cart_dist < self.tolerance and self.next_point_index < len(self.path_poses) - 1:
            next_point = self.path_poses[self.next_point_index + 1]
            cart_dist = np.sqrt((robot_pose.x - next_point.x) ** 2 + (robot_pose.y - next_point.y) ** 2)
            self.next_point_index += 1
        if self.next_point_index == len(self.path_poses) - 1:
            return True
        else:
            return False

    def calc_traj_points(self, transform):
        self.candidate_traj_points = self.traj_roll_out.get_real_world_points(transform)
    
    def choose_trajectory(self):
        self.calc_cost_list()
        min_cost = min(self.cost_list)
        min_index = self.cost_list.index(min_cost)
        return min_index    

    def calc_cost_list(self):
        self.cost_list = []
        for i in range(len(self.candidate_traj_points)):
            self.cost_list.append(self.calc_cost(i))
    
    def calc_cost(self, ith_path):
        return self.collision_cost(ith_path)

    def collision_cost(self, ith_path: int):
        path_points = self.candidate_traj_points[ith_path]
        print(f"{ith_path}th path_points:", path_points)
        for i in range(len(path_points)):
            x, y = self.map_util.act_pos_to_grid_pos(path_points[i][0][0], path_points[i][1][0])
            if self.map_util.occupancy_check_cost_map_grid_coord(x, y):
                print(f"{ith_path}th collision", i)
                return self.coll_k * (len(path_points) - i) ** self.coll_m + self.coll_b
        return 0 * (len(path_points) - i) ** self.coll_m + self.coll_b

    def turn_penalty(self, ith_path: int):
        '''
        penalize turning in the trajectory
        '''
        w = self.candidate_vels[ith_path][1]
        return self.turn_k * abs(w) ** self.turn_m

class TestProgram:
    def __init__(self):
        self.next_point_pub = rospy.Publisher('/next_point', PointStamped, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.robot_pose = None

    def odom_callback(self, msg):
        self.robot_pose = msg.pose.pose

    def publish_next_point(self, next_point: Point):
        pub_point = PointStamped()
        pub_point.header.frame_id = "map"
        pub_point.header.stamp = rospy.Time.now()
        pub_point.point = next_point
        pub_point.point.z = 0.3
        self.next_point_pub.publish(pub_point)

class PointListPublisher:
    def __init__(self):
        self.marker_pub = rospy.Publisher('point_list_marker', Marker, queue_size=10)

    def publish_point_list(self, point_list):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "points"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.color.a = 0.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.points = point_list

        self.marker_pub.publish(marker)

goal_point = Point()
plan = False
def clicked_callback(msg):
    global goal_point
    global plan
    goal_point = msg.point
    plan = True
if __name__=="__main__":
    rospy.init_node("cost_function_p")

    dt = 0.05
    rate = rospy.Rate(1/dt)

    path_pub = rospy.Publisher('path_topic', Path, queue_size=10)
    point_list_publisher = PointListPublisher()
    clicked_sub = rospy.Subscriber('clicked_point', PointStamped, clicked_callback)

    test_program = TestProgram()


    path_msg = Path()
    path_msg.header.frame_id = "map"
    path_msg.header.stamp = rospy.Time.now()
    
    folder_path = "/home/sentry_train_test/AstarTraining/sim_nav/src/bot_sim/scripts/"
    rrt = RRTStar(folder_path + "RRT/CostMap/CostMapR0d5")

    # Generate trajectory points
    lin_vel = 0.2
    traj_vel = [[lin_vel, 0.0]]
    for w in range(1, 10):
        traj_vel.append([lin_vel,  w*0.05])
        traj_vel.append([lin_vel, -w*0.05])
    cost_function = CostFunction(folder_path + "DWA/CostMap/CostMapR0d3",traj_vel)

    tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) # tf buffer length
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    path_exist = False
    
    cum_time = 0
    while not rospy.is_shutdown():
        if plan == True:
            plan = False
            path, mesg = rrt.rrt_plan_selection(test_program.robot_pose.position, goal_point)
            if mesg == "found":
                path_exist = True
                cost_function.set_path(path)

                rospy.loginfo("path found")
                path_msg.poses = []
                for i in range(len(path)):
                    pose_stamped = PoseStamped()
                    pose_stamped.header.frame_id = "map"
                    pose_stamped.header.stamp = rospy.Time.now()
                    pose_stamped.pose.position.x = path[i].x
                    pose_stamped.pose.position.y = path[i].y
                    q = quaternion_from_euler(0, 0, path[i].z)
                    pose_stamped.pose.orientation.x = q[0]
                    pose_stamped.pose.orientation.y = q[1]
                    pose_stamped.pose.orientation.z = q[2]
                    pose_stamped.pose.orientation.w = q[3]
                    path_msg.poses.append(pose_stamped)
                path_pub.publish(path_msg)
            else:
                rospy.loginfo("path not found")
        if path_exist == True:
            if test_program.robot_pose != None:
                goal_reached = cost_function.check_next_point_update(test_program.robot_pose.position)
                next_point = cost_function.path_poses[cost_function.next_point_index]
                test_program.publish_next_point(next_point)
                if goal_reached == True:
                    rospy.loginfo("goal reached!")
                    path_exist = False

        if cum_time >= 0.5 and path_exist == True:
            cum_time = 0
            try:
                trans = tf_buffer.lookup_transform('odom', 'base_link', rospy.Time.now(), rospy.Duration(1.0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                pass
            cost_function.calc_traj_points(trans.transform)
            i = cost_function.choose_trajectory()
            print("best",i)
            print(cost_function.cost_list)
            traj_list = cost_function.candidate_traj_points[i]
            point_list = []
            for i in range(len(traj_list)):
                point = Point()
                point.x = traj_list[i][0][0]
                point.y = traj_list[i][1][0]
                point_list.append(point)
            point_list_publisher.publish_point_list(point_list)
        
        cum_time += dt
        rate.sleep()



