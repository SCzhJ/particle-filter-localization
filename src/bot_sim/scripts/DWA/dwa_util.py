#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
import numpy as np
import pickle
from typing import List, Tuple

class MapUtil:
    def __init__(self, occ_threshold: int = 95):
        """Initialize the MapUtil class."""
        self.map_frame = "map"
        self.odom_frame = "odom"
        self.grid_map = None
        self.grid_info = None
        self.cost_map = None
        self.load_map()
        # map occupancy probability from 0 to 100, set a threshold to determine occupancy
        self.occ_threshold = occ_threshold

    def load_cost_map(self, cost_map_file: str):
        """Load the cost map from a file."""
        with open(cost_map_file, 'rb') as f:
            self.cost_map = pickle.load(f)

    def act_pos_to_grid_pos(self, x: float, y: float) -> Tuple[int, int]:
        """Take world coordinate, return grid pixel position."""
        x -= self.grid_info.origin.position.x
        y -= self.grid_info.origin.position.y
        x = int(x / self.grid_info.resolution)
        y = int(y / self.grid_info.resolution)
        return x, y

    def grid_pos_to_act_pos(self, x: int, y: int) -> Tuple[float, float]:
        """Take grid pixel position, return world coordinate."""
        x *= self.grid_info.resolution
        y *= self.grid_info.resolution
        x += self.grid_info.origin.position.x
        y += self.grid_info.origin.position.y
        return x, y

    def occupancy_check_grid_coord(self, x: int, y: int) -> bool:
        """Check occupancy inputing grid coordinate."""
        if x > self.grid_info.width or y > self.grid_info.height or x < 0 or y < 0:
            return True
        occ = self.grid_map[y * self.grid_info.width + x]
        return occ == -1 or occ > self.occ_threshold

    def occupancy_check_cost_map_grid_coord(self, x: int, y: int) -> bool:
        """Check occupancy in cost map inputing grid coordinate."""
        if x > self.grid_info.width or y > self.grid_info.height or x < 0 or y < 0:
            return True
        occ = self.cost_map[y * self.grid_info.width + x]
        return (occ == -1 or occ > self.occ_threshold)

    def get_map(self):
        """Return 1D grid map."""
        return self.grid_map

    def get_map_info(self):
        """Return map info."""
        return self.grid_info

    def load_map(self):
        """Load map."""
        rospy.wait_for_service('/static_map') 
        grid_map_service = rospy.ServiceProxy('/static_map', GetMap)
        response = grid_map_service()
        rospy.loginfo("map loaded")
        self.grid_map = response.map.data
        self.grid_info = response.map.info
