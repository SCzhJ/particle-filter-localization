#!/usr/bin/env python3

import rospy
import util
from sklearn.neighbors import KDTree
import pickle
from nav_msgs.msg import OccupancyGrid
import util
import numpy as np

class LikelihoodFieldGenerator:
    def __init__(self,sigma=0.5):
        self.util = util.Util()
        self.grid_map = self.util.getMap()
        self.grid_info = self.util.getMapInfo()
        self.occ_points = []
        self.all_points = []
        self.OrganizePoints()

        self.sigma = sigma
        self.kdt = self.GenKDTree()
        self.likelihood_field = list(self.grid_map)

    def getLikelihoodField(self):
        return self.likelihood_field

    def StoreLikelihoodField(self,name_of_file):
        with open(name_of_file,'wb') as f:
            pickle.dump(self.likelihood_field,f)

    def GenLikelihoodField(self):
        # the following returns numpy array n*1 of distances 
        dists = self.kdt.query(self.all_points,k=1)[0][:]
        # the following returns numpy array n*1 of likelihoods
        prob = np.exp(-(dists**2)/(2*self.sigma**2))
        print("prob shape:",np.shape(prob))

        # find minimum probability that's not zero and replace zeros with it 
        min_prob=1
        for i in range(len(prob)):
            if prob[i][0]!=0 and prob[i][0]<min_prob:
                min_prob = prob[i][0]
        for i in range(len(prob)):
            if prob[i][0]==0:
                prob[i][0] = min_prob

        # assign likelihood to the field map
        for i in range(len(self.all_points)):
            x = self.all_points[i][0]
            y = self.all_points[i][1]
            self.likelihood_field[y*self.grid_info.width+x] = prob[i][0]
        print("field shape",np.shape(self.likelihood_field))

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

    file_name = "LikelihoodField/likelihood_field_sigma_0_5"
    likelihood_field = LikelihoodFieldGenerator(sigma=0.5)
    likelihood_field.GenLikelihoodField()
    print("field generated")
    field = likelihood_field.getLikelihoodField()
    likelihood_field.StoreLikelihoodField(file_name)
    print("field saved")

    # with open(file_name,'rb') as f:
        # field = pickle.load(f)

    rate = rospy.Rate(5)

    pub = rospy.Publisher("/modified_map", OccupancyGrid, queue_size=10)

    util = util.Util()

    occupancy_grid = OccupancyGrid()
    occupancy_grid.header.frame_id = "map"
    occupancy_grid.info = util.grid_info
    for i in range(len(field)):
        field[i] = int(field[i]*100)
    occupancy_grid.data = field

    print("enter loop")

    while not rospy.is_shutdown():
        occupancy_grid.header.stamp = rospy.Time.now()
        pub.publish(occupancy_grid)

        rate.sleep()













