#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
import numpy as np
import pickle
from typing import List, Tuple
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PointStamped, Twist, PoseArray, Pose, Point
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class TreeNode:
    def __init__(self, index: int, point: Point):
        self.index = index
        self.point = point
        self.parent_index = -1
        self.children_indices = []

    def modify_parent_index(self, i: int):
        """Modify the parent index of the node."""
        self.parent_index = i
    
    def get_parent_index(self) -> int:
        """Return the parent index."""
        return self.parent_index

    def get_children_num(self) -> int:
        """Return the number of children."""
        return len(self.children_indices)

    def get_children_index(self, i: int) -> int:
        """Return the index of the ith child."""
        return self.children_indices[i]
    
    def add_children_index(self, i: int):
        """Add the index of the ith child."""
        self.children_indices.append(i)

    def delete_children_index(self, i: int):
        """Delete the index of the ith child."""
        self.children_indices.remove(i)


class TreeUtil:
    def __init__(self, rootPoint: Point = Point(0,0,0)):
        self.root_node = TreeNode(0, rootPoint)
        self.tree = [self.root_node]

    def get_node(self, i: int) -> TreeNode:
        """Return the ith node."""
        return self.tree[i]

    def add_node(self, point: Point, parentIndex: int) -> int:
        """Add a node to the tree."""
        self.tree[parentIndex].children_indices.append(len(self.tree))
        new_node = TreeNode(len(self.tree), point)
        new_node.modify_parent_index(parentIndex)
        self.tree.append(new_node)
        return len(self.tree) - 1

    def traverse_node_add_to_line_list(self, Nodei: int, LineList: List[Point]) -> List[Point]:
        """Traverse the tree and add nodes to a line list."""
        Node = self.get_node(Nodei)
        child_num = Node.get_children_num()
        if child_num == 0:
            return LineList
        for i in range(child_num):
            child_index = Node.get_children_index(i)
            LineList.append(Node.point)
            LineList.append(self.tree[child_index].point)
            LineList = self.traverse_node_add_to_line_list(child_index, LineList)
        return LineList

    def traverse_node_add_to_debug_line_list(self, Nodei: int, DebugLineList: List[Point]) -> List[Point]:
        """Traverse the tree and add nodes to a debug line list."""
        Node = self.get_node(Nodei)
        child_num = Node.get_children_num()
        if child_num == 0:
            return DebugLineList
        for i in range(child_num):
            child_index = Node.get_children_index(i)
            # Add your code here

class MapUtil:
    def __init__(self, cost_map_file: str = "", occ_threshold: int = 95):
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
    
    def occupancy_value_check_cost_map_grid_coord(self, x: int, y: int) -> bool:
        """Check occupancy in cost map inputing grid coordinate."""
        if x > self.grid_info.width or y > self.grid_info.height or x < 0 or y < 0:
            rospy.logerr("Invalid grid coordinate")
            return -1
        occ = self.cost_map[y * self.grid_info.width + x]
        if occ == -1:
            return 100
        else:
            return occ

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
        self.origin = response.map.info.origin
        self.origin_x = response.map.info.origin.position.x
        self.origin_y = response.map.info.origin.position.y
class OdomSubscriber:
    def __init__(self):
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.robot_pose = None

    def odom_callback(self, msg):
        self.robot_pose = msg.pose.pose

    def get_pose(self):
        if self.robot_pose is None:
            return None, None, None
        x = self.robot_pose.position.x
        y = self.robot_pose.position.y
        rospy.loginfo("get information: x %s y %s", x, y)
        theta = euler_from_quaternion([self.robot_pose.orientation.x, 
                                       self.robot_pose.orientation.y, 
                                       self.robot_pose.orientation.z, 
                                       self.robot_pose.orientation.w])[2]
        return x, y, theta
