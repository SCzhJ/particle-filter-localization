#!/usr/bin/env python3

from typing import Any
import rospy
from dwa_util import *
import copy


class CostFunction:
    def __init__(self, cost_map_path: str, dynamic_map_name: str,
                 iteration: int, pathfollow_iter: int, obstacle_iter: int):
        # static obstacle map
        self.stat_obs_map_util = MapUtil()
        rospy.loginfo("got static map name: %s", cost_map_path)
        self.stat_obs_map_util.load_cost_map(cost_map_path)

        # dynamic obstacle map
        self.dyn_obs_map_util = MapUtil()
        rospy.loginfo("got dynamic map name: %s", dynamic_map_name)
        self.dyn_obs_map_util.subscribe_map(dynamic_map_name)

        # iterations
        self.iteration = iteration
        self.pathfollow_iter = pathfollow_iter
        self.obstacle_iter = obstacle_iter

        # Naive Cost Function:
        # total_cost = dynamic_obstacle_cost + static_obstacle_cost + path_following_cost + turn_cost + bearing_cost

        # dynamic_obstacle_cost = k_d * (N - i) ^ m_d, where N is the number of points in the trajectory, 
        # i is the index of the first point to meet collision. No collision return 0
        self.k_d = 100
        self.m_d = 1.5

        # static_obstacle_cost = k_o * (N - i) ^ m_o, where N is the number of points in the trajectory, 
        # i is the index of the first point to meet collision. No collision return 0
        self.k_o = 7
        self.m_o = 2

        # path_following_cost = k_p * (dist) ^ m_p
        # where dist is the distance between the end point of trajectory and the next point in the path
        self.k_p = 3.0
        self.m_p = 1.5

        # turn_cost = k_t * |theta| ^ m_t
        self.k_t = 0.3
        self.m_t = 0.2

        # bearing_cost = k_b * |theta_difference| ^ m_b
        self.k_b = 0.9
        self.m_b = 0.3

        # Want to take in account the cost of neighboring trajectories
    
    def avg_cost_from_traj(self, start: int, end: int, traj_i: int, cost_list: List[float]) -> float:
        '''
        start: int
        end: int
        traj_i: int
        cost_list: List[float]
        '''
        return sum(cost_list[start:end]) / abs(end - start)
    
    def total_cost(self, world_frame_traj_points: List, 
                   robot_frame_traj_points, next_point: Point,
                   scaler: float = 1.0) -> float:
        '''
        traj_points: list of np.ndarray[3, 1]
        next_point: Point
        '''
        return self.dynamic_obstacle_cost(robot_frame_traj_points, self.obstacle_iter) + \
               self.static_obstacle_cost(world_frame_traj_points,self.obstacle_iter) + \
               self.path_following_cost(scaler, world_frame_traj_points, next_point,self.pathfollow_iter) + \
               self.turn_cost(world_frame_traj_points,self.iteration) + \
               self.bearing_cost(world_frame_traj_points, next_point, self.iteration)

    def total_cost_omni(self, world_frame_traj_points: List, 
                   robot_frame_traj_points: List, next_point: Point,
                   scaler: float = 1.0) -> float:
        '''
        traj_points: list of np.ndarray[3, 1]
        next_point: Point
        '''
        rospy.loginfo("omni cost")
        return self.dynamic_obstacle_cost(robot_frame_traj_points, self.obstacle_iter) + self.path_following_cost(world_frame_traj_points, next_point,self.pathfollow_iter, scaler)
    
    def constant_cost(self, k):
        return k
    
    def bearing_cost(self, traj_points: List, next_point: Point, iteration_num: int) -> float:
        '''
        traj_points: list of np.ndarray[3, 1]
        next_point: Point
        '''
        dx = next_point.x - traj_points[iteration_num-1][0][0]
        dy = next_point.y - traj_points[iteration_num-1][1][0]
        angle = np.arctan2(dy, dx)
        theta_difference = abs(traj_points[iteration_num-1][2][0] - angle)
        return self.k_b * theta_difference ** self.m_b
    
    def dynamic_obstacle_cost(self, robot_frame_traj, dyn_obs_iter) -> float:
        for i in range(dyn_obs_iter):
            x, y = self.dyn_obs_map_util.act_pos_to_grid_pos(copy.copy(robot_frame_traj[i][0][0]), 
                                                     copy.copy(robot_frame_traj[i][1][0]))
            occ_cost = self.dyn_obs_map_util.occupancy_value_check_grid_coord(x, y)

            if occ_cost > 20:
                return self.k_d * (occ_cost/100 * (len(robot_frame_traj) - i)) ** self.m_d
        return 0
    
    def static_obstacle_cost(self, traj_points: List, static_obs_iter: int) -> float:
        '''
        traj_points: list of np.ndarray[3, 1]
        '''
        for i in range(static_obs_iter):
            x, y = self.stat_obs_map_util.act_pos_to_grid_pos(copy.copy(traj_points[i][0][0]), 
                                                     copy.copy(traj_points[i][1][0]))
            occ_cost = self.stat_obs_map_util.occupancy_check_cost_map_grid_coord(x, y)
            if occ_cost > 20:
                return self.k_o * (occ_cost/100 * (len(traj_points) - i)) ** self.m_o
        return 0
    
    def path_following_cost(self, traj_points: List, next_point: Point, path_follow_iter: int, scaler: float = 1.0) -> float:
        '''
        traj_points: list of np.ndarray[3, 1]
        next_point: Point
        '''
        dist = np.sqrt((traj_points[path_follow_iter][0][0] - next_point.x) ** 2 + (traj_points[path_follow_iter][1][0] - next_point.y) ** 2)
        return self.k_p * (dist/scaler) ** self.m_p
    
    def turn_cost(self, traj_points: List, turn_iter: int) -> float:
        '''
        traj_points: list of np.ndarray[3, 1]
        '''
        theta = abs(traj_points[turn_iter][2][0])
        return self.k_t * abs(theta) ** self.m_t



# Test Program
if __name__ == "__main__":
    rospy.init_node("cost_function_node")