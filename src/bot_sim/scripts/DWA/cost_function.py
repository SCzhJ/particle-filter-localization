#!/usr/bin/env python3

from typing import Any
import rospy
from traj import *
from dwa_util import *


class CostFunction:
    def __init__(self, cost_map_path: str, dynamic_map_name: str):
        # static obstacle map
        self.stat_obs_map_util = MapUtil()
        self.stat_obs_map_util.load_cost_map(cost_map_path)

        # dynamic obstacle map
        self.dyn_obs_map_util = MapUtil()
        self.dyn_obs_map_util.subscribe_map(dynamic_map_name)

        # Naive Cost Function:
        # total_cost = dynamic_obstacle_cost + static_obstacle_cost + path_following_cost + turn_cost + bearing_cost

        # dynamic_obstacle_cost = k_d * (N - i) ^ m_d, where N is the number of points in the trajectory, 
        # i is the index of the first point to meet collision. No collision return 0
        self.k_d = 20
        self.m_d = 1.5

        # static_obstacle_cost = k_o * (N - i) ^ m_o, where N is the number of points in the trajectory, 
        # i is the index of the first point to meet collision. No collision return 0
        self.k_o = 9
        self.m_o = 2

        # path_following_cost = k_p * (dist) ^ m_p
        # where dist is the distance between the end point of trajectory and the next point in the path
        self.k_p = 2.0
        self.m_p = 2

        # turn_cost = k_t * |theta| ^ m_t
        self.k_t = 0.01
        self.m_t = 0.3

        # bearing_cost = k_b * |theta_difference| ^ m_b
        self.k_b = 0.6
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
    
    def total_cost(self, world_frame_traj_points: List, robot_frame_traj_points, next_point: Point, iteration_num: int) -> float:
        '''
        traj_points: list of np.ndarray[3, 1]
        next_point: Point
        '''
        return self.dynamic_obstacle_cost(robot_frame_traj_points) + \
               self.static_obstacle_cost(world_frame_traj_points) + \
               self.path_following_cost(world_frame_traj_points, next_point) + \
               self.turn_cost(world_frame_traj_points) + \
               self.bearing_cost(world_frame_traj_points, next_point, iteration_num)
    
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
    
    def dynamic_obstacle_cost(self, robot_frame_traj) -> float:
        for i in range(len(robot_frame_traj)):
            x, y = self.dyn_obs_map_util.act_pos_to_grid_pos(copy.copy(robot_frame_traj[i][0][0]), 
                                                     copy.copy(robot_frame_traj[i][1][0]))
            occ_cost = self.dyn_obs_map_util.occupancy_value_check_grid_coord(x, y)
            if occ_cost > 20:
                return self.k_d * (occ_cost/100 * (len(robot_frame_traj) - i)) ** self.m_d
        return 0
    
    def static_obstacle_cost(self, traj_points: List) -> float:
        '''
        traj_points: list of np.ndarray[3, 1]
        '''
        for i in range(len(traj_points)):
            x, y = self.stat_obs_map_util.act_pos_to_grid_pos(copy.copy(traj_points[i][0][0]), 
                                                     copy.copy(traj_points[i][1][0]))
            occ_cost = self.stat_obs_map_util.occupancy_check_cost_map_grid_coord(x, y)
            if occ_cost > 20:
                return self.k_o * (occ_cost/100 * (len(traj_points) - i)) ** self.m_o
        return 0
    
    def path_following_cost(self, traj_points: List, next_point: Point) -> float:
        '''
        traj_points: list of np.ndarray[3, 1]
        next_point: Point
        '''
        dist = np.sqrt((traj_points[-1][0][0] - next_point.x) ** 2 + (traj_points[-1][1][0] - next_point.y) ** 2)
        return self.k_p * dist ** self.m_p
    
    def turn_cost(self, traj_points: List) -> float:
        '''
        traj_points: list of np.ndarray[3, 1]
        '''
        theta = abs(traj_points[-1][2][0])
        return self.k_t * abs(theta) ** self.m_t

