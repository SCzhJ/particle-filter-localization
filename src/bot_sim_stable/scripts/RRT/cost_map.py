#!/usr/bin/env python3

import rospy
from rrt_util import *
from sklearn.neighbors import KDTree
import pickle
from nav_msgs.msg import OccupancyGrid
import numpy as np

class CostMap:
    def __init__(self, robot_radius: float = 0.5):
        """Initialize the CostMap class."""
        self.util = MapUtil()
        self.grid_map = self.util.get_map()
        self.grid_info = self.util.get_map_info()
        self.occ_points = []
        self.all_points = []
        self.organize_points()
        self.kdt = self.gen_kd_tree()
        self.cost_map = list(self.grid_map)
        self.close_to_obstacle= 0.05
        self.near_to_obstacle = robot_radius

    def get_cost_map(self):
        """Return the cost map."""
        return self.cost_map

    def store_cost_map(self, name_of_file: str):
        """Store the cost map in a file."""
        with open(name_of_file, 'wb') as f:
            pickle.dump(self.cost_map, f)

    def gen_cost_map(self):
        """Generate the cost map."""
        # the following returns numpy array n*1 of distances 
        dists = self.kdt.query(self.all_points, k=1)[0][:]
        grid_dist = self.near_to_obstacle / self.util.grid_info.resolution
        close_dist = self.close_to_obstacle / self.util.grid_info.resolution
        for i in range(len(self.all_points)):
            if dists[i][0] < grid_dist:
                x = self.all_points[i][0]
                y = self.all_points[i][1]
                if dists[i][0] > close_dist:
                    self.cost_map[y * self.grid_info.width + x] = 70
                else:
                    self.cost_map[y * self.grid_info.width + x] = 90
        rospy.loginfo("cost map shape %s", np.shape(self.cost_map))

    def gen_kd_tree(self):
        """Generate a KDTree."""
        return KDTree(self.occ_points)

    def organize_points(self):
        """Organize the points."""
        self.occ_points = []
        self.all_points = []
        for x in range(self.grid_info.width):
            for y in range(self.grid_info.height):
                self.all_points.append([x, y])
                if self.grid_map[y * self.grid_info.width + x] != -1 and \
                self.util.occupancy_check_grid_coord(x, y):
                    self.occ_points.append([x, y])

if __name__=="__main__":
    rospy.init_node("cost_map_p")

    file_name = "CostMap/CostMapR05C005"
    cost_map = CostMap(robot_radius=0.5)
    cost_map.gen_cost_map()
    rospy.loginfo("cost map generated")
    cost_map.store_cost_map(file_name)
    rospy.loginfo("cost map saved")

    with open(file_name,'rb') as f:
        my_cost_map = pickle.load(f)

    rate = rospy.Rate(5)

    pub = rospy.Publisher("/modified_map", OccupancyGrid, queue_size=10)

    util = MapUtil()

    occupancy_grid = OccupancyGrid()
    occupancy_grid.header.frame_id = "map"
    occupancy_grid.info = util.get_map_info()
    occupancy_grid.data = my_cost_map

    rospy.loginfo("enter loop")

    while not rospy.is_shutdown():
        occupancy_grid.header.stamp = rospy.Time.now()
        pub.publish(occupancy_grid)

        rate.sleep()