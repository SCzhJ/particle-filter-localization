#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np
from rrt_util import TreeNode, MapUtil, TreeUtil

class RRT:
    def __init__(self,Qinit=Point(0,0,0)):
        # utility
        self.map_util = MapUtil()
        # the below pictures the sampling area
        # square area, x from -12 to 12, y from -12 to 12
        # coordinate of odometry 
        self.sampling_origin = [-9, -9]
        self.sampling_range  = [19, 18]
        
        # security iteration limit
        self.max_iter = 100000

        # q denoting configuration
        self.q_init = Qinit

        # RRT params
        self.step_size = 0.5

        # Tree
        self.Tree = TreeUtil(Qinit)

        # step size for collision detection 
        # let it be 1/n meaning detecting n 
        # segments within the step
        self.coll_step= 0.1

    # Generate random configuration
    def RandConf(self):
        rand_x = self.sampling_origin[0] + np.random.rand() * self.sampling_range[0]
        rand_y = self.sampling_origin[1] + np.random.rand() * self.sampling_range[1]
        return rand_x, rand_y



  # Generating Random Configuration, rejecting the samples at obstacle
    def RandConfRej(self):
        done = False
        counter = 0
        while (not done) and counter != self.max_iter:
            x, y = self.RandConf()
            if not self.ObstacleDetection(x, y):
                done = True
            counter += 1
        if counter == self.max_iter:
            rospy.logerr("SAMPLING ITERATION REACHED MAXIMUM!")
        return x, y

    # Detect obstacle inputing real coordinate
    def ObstacleDetection(self, real_coord_x, real_coord_y):
        grid_x, grid_y = self.map_util.ActPos2GridPos(real_coord_x, real_coord_y)
        return self.map_util.OccupancyCheckGridCoord(grid_x, grid_y)

    def NearestNeighbor(self, x, y):
        nearest_dist  = 100000
        nearest_index = -1
        for i in range(len(self.Tree.tree)):
            point = self.Tree.getNode(i).point
            new_dist = np.sqrt((point.x-x)**2 + (point.y-y)**2)
            if new_dist < nearest_dist:
               nearest_index = i
               nearest_dist = new_dist
        return nearest_index

    # Detect if collision exist between points
    def CollisionFree(self, x, y, x_d, y_d):
        x_del = x_d - x
        y_del = y_d - y
        vec_len = np.sqrt(x_del**2+y_del**2)
        x_del = self.coll_step * self.step_size * x_del / vec_len
        y_del = self.coll_step * self.step_size * y_del / vec_len
        length = 0
        while length<self.step_size:
            x += x_del
            y += y_del
            if self.ObstacleDetection(x, y):
                return False
            length += self.coll_step * self.step_size
        return True

    
    def Step(self, nearest_index, new_x, new_y):
        nearest_point = self.Tree.getNode(nearest_index).point
        x_del = new_x - nearest_point.x
        y_del = new_y - nearest_point.y
        vec_len = np.sqrt(x_del**2+y_del**2)
        x_del = self.step_size * x_del/vec_len
        y_del = self.step_size * y_del/vec_len

        x_new = nearest_point.x + x_del
        y_new = nearest_point.y + y_del

        return x_new, y_new


if __name__=="__main__":
    rospy.init_node("RRT_test")
    rate = rospy.Rate(10)

    rrt = RRT()

    pub = rospy.Publisher("tree_marker", Marker, queue_size=10)

    counter = 0

    # Marker params
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "marker_tree"
    marker.id = 0
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    marker.pose.orientation.x = 0
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = 0
    marker.pose.orientation.w = 1
    marker.lifetime = rospy.Duration()

    while (not rospy.is_shutdown()) and counter < 600:
        x, y = rrt.RandConfRej()
        new_point = Point(x,y,0)
        nearest_i = rrt.NearestNeighbor(x,y)
        nearest_point = rrt.Tree.getNode(nearest_i).point
        x_new, y_new = rrt.Step(nearest_i, x, y)
        if rrt.CollisionFree(nearest_point.x, nearest_point.y, x_new, y_new):
            rrt.Tree.AddNode(Point(x_new, y_new, 0), nearest_i)
            LineList = rrt.Tree.TraverseNodeAddToLineList(0,[])
            marker.points = LineList
            pub.publish(marker)
        counter += 1
        rate.sleep()

    rospy.spin()




