#!/usr/bin/env python3

import rospy
from rrt_util import *
from sklearn.neighbors import KDTree
import pickle
from nav_msgs.msg import OccupancyGrid
import numpy as np
import copy

class CostMap:
    def __init__(self,robotRadius=0.5):
        self.util = MapUtil()
        self.grid_map = self.util.getMap()
        self.grid_info = self.util.getMapInfo()
        self.occ_points = []
        self.all_points = []
        self.OrganizePoints()
        self.kdt = self.GenKDTree()
        self.cost_map = list(self.grid_map)

        self.robot_radius = robotRadius

    def getCostMap(self):
        return self.cost_map

    def StoreCostMap(self,name_of_file):
        with open(name_of_file,'wb') as f:
            pickle.dump(self.cost_map,f)

    def GenCostMap(self):
        # the following returns numpy array n*1 of distances 
        dists = self.kdt.query(self.all_points,k=1)[0][:]
        grid_dist = self.robot_radius / self.util.grid_info.resolution
        for i in range(len(self.all_points)):
            if dists[i][0] < grid_dist:
                x = self.all_points[i][0]
                y = self.all_points[i][1]
                self.cost_map[y*self.grid_info.width+x] = 100
        print("cost map shape",np.shape(self.cost_map))


    def GenKDTree(self):
        return KDTree(self.occ_points)

    def OrganizePoints(self):
        self.occ_points = []
        self.all_points = []
        for x in range(self.grid_info.width):
            for y in range(self.grid_info.height):
                self.all_points.append([x,y])
                if self.grid_map[y*self.grid_info.width+x]!=-1 and \
                self.util.OccupancyCheckGridCoord(x,y):
                    self.occ_points.append([x,y])

# Test Program For LikelihoodFieldGenerator
if __name__=="__main__":
    rospy.init_node("likelihood_field_p")

    file_name = "CostMap/CostMapR0d5"
    # cost_map = CostMap(robotRadius=0.5)
    # cost_map.GenCostMap()
    # print("cost map generated")
    # cost_map.StoreCostMap(file_name)
    # print("cost map saved")

    with open(file_name,'rb') as f:
        MyCostMap = pickle.load(f)

    rate = rospy.Rate(5)

    pub = rospy.Publisher("/modified_map", OccupancyGrid, queue_size=10)

    util = MapUtil()

    occupancy_grid = OccupancyGrid()
    occupancy_grid.header.frame_id = "map"
    occupancy_grid.info = util.grid_info
    occupancy_grid.data = MyCostMap

    print("enter loop")

    while not rospy.is_shutdown():
        occupancy_grid.header.stamp = rospy.Time.now()
        pub.publish(occupancy_grid)

        rate.sleep()













