#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PointStamped
import numpy as np
from rrt_util import TreeNode, MapUtil, TreeUtil
import pickle

class RRT_star:
    def __init__(self, Qinit=Point(0,0,0)):
        # utility
        self.map_util = MapUtil()
        # the below pictures the sampling area
        # square area, x from -12 to 12, y from -12 to 12
        # coordinate of odometry 
        self.sampling_origin = [-9, -9]
        self.sampling_range  = [19, 18]
        self.sampling_radius = 30

        self.normal_sigma = 0.7
        self.normal_enlargement = 20
        
        # security iteration limit
        self.max_iter = 10000

        # q denoting configuration
        self.q_init = Qinit

        # RRT params
        self.step_size = 0.1
        self.gamma     = 4
        self.eta       = 0.4 * self.gamma
        self.d         = 2
        self.default_goal_radius = 0.3
        self.goal_radius = self.default_goal_radius
        self.max_search_iter = 10000

        # step size for collision detection 
        # let it be 1/n meaning detecting n 
        # segments within the step
        self.coll_step= 0.1

        # Tree
        self.Tree = TreeUtil(Qinit)
    
    def InitTree(self):
        self.Tree = TreeUtil(self.q_init)
    
    def setGoalRadius(self, r):
        self.goal_radius = r
    
    def RandConfAtGoalGaussian(self, goal_x, goal_y):
        rand_range = np.random.normal(self.normal_sigma) * self.normal_enlargement
        rand_theta = np.random.rand() * 2 * np.pi
        rand_x = np.cos(rand_theta) * rand_range + goal_x
        rand_y = np.sin(rand_theta) * rand_range + goal_y
        return rand_x, rand_y
    
    def RandConfCircle(self):
        rand_range = np.random.rand() * self.sampling_radius
        rand_theta = np.random.rand() * 2 * np.pi
        rand_x = np.cos(rand_theta) * rand_range
        rand_y = np.sin(rand_theta) * rand_range
        return rand_x, rand_y

    # Generate random configuration
    def RandConfRect(self):
        rand_x = self.sampling_origin[0] + np.random.rand() * self.sampling_range[0]
        rand_y = self.sampling_origin[1] + np.random.rand() * self.sampling_range[1]
        return rand_x, rand_y

    # Generating Random Configuration, rejecting the samples at obstacle
    def RandConfRecRej(self):
        done = False
        counter = 0
        while (not done) and counter != self.max_iter:
            x, y = self.RandConfRect()
            if not self.ObstacleDetection(x, y):
                done = True
            counter += 1
        if counter == self.max_iter:
            rospy.logerr("SAMPLING ITERATION REACHED MAXIMUM!")
        return x, y

    # Detect obstacle inputing real coordinate
    def ObstacleDetection(self, real_coord_x, real_coord_y):
        grid_x, grid_y = self.map_util.ActPos2GridPos(real_coord_x, real_coord_y)
        return self.map_util.OccupancyCheckCostMapGridCoord(grid_x, grid_y)

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

    # Return a list of neighboring indices
    def Neighbors(self, x, y):
        cardV = len(self.Tree.tree)
        radius = min(self.gamma*(np.log(cardV)/cardV)**(1/self.d),self.eta)
        neighbor_list = []
        for i in range(len(self.Tree.tree)):
            point = self.Tree.getNode(i).point
            dist = np.sqrt((x-point.x)**2+(y-point.y)**2)
            if dist < radius:
                neighbor_list.append(i)
        return neighbor_list
    
    # cost of a line
    def c(self, x_1, y_1, x_2, y_2):
        return np.sqrt((x_1-x_2)**2+(y_1-y_2)**2)
    
    # cost of a path to a point
    def Cost(self, node_i):
        cumulative_cost = 0
        current_node = self.Tree.getNode(node_i)
        parent_index = current_node.getParentIndex()
        while parent_index != None: 
            parent_node = self.Tree.getNode(parent_index)
            cumulative_cost += self.c(current_node.point.x,
                                      current_node.point.y,
                                      parent_node.point.x,
                                      parent_node.point.y)
            current_node = parent_node
            parent_index = current_node.getParentIndex()
        return cumulative_cost
    
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
    
    def GoalReached(self, goal_x, goal_y, new_x, new_y):
        if self.c(goal_x, goal_y, new_x, new_y) < self.goal_radius:
            return True
        else:
            return False
        
    def RRT_plan(self, x_goal, y_goal):
        if self.ObstacleDetection(x_goal, y_goal):
            rospy.logerr("RRT Goal In Obstacle Region!")
            return None, "Goal in Obstacle"
        self.InitTree()
        goal_reached = False
        counter = 0
        while counter < self.max_search_iter and goal_reached == False:
            x, y = self.RandConfAtGoalGaussian(x_goal, y_goal)
            new_point = Point(x,y,0)
            nearest_i = self.NearestNeighbor(x,y)
            nearest_point = self.Tree.getNode(nearest_i).point
            x_new, y_new = self.Step(nearest_i, x, y)
            if self.CollisionFree(nearest_point.x, nearest_point.y, x_new, y_new):
                neighbor_index_list = self.Neighbors(x_new, y_new)
                min_i = nearest_i
                min_cost = self.Cost(nearest_i)+self.c(nearest_point.x,nearest_point.y,x_new,y_new)

                for i in neighbor_index_list:
                    near_point = self.Tree.getNode(i).point
                    if self.CollisionFree(near_point.x,near_point.y,x_new,y_new) and \
                    self.Cost(i)+self.c(near_point.x,near_point.y,x_new,y_new)<min_cost:
                        min_i = i
                        min_cost = self.Cost(i)+self.c(near_point.x,near_point.y,x_new,y_new)

                new_i = self.Tree.AddNode(Point(x_new, y_new, 0), min_i)
                for i in neighbor_index_list:
                    near_point = self.Tree.getNode(i).point
                    if self.CollisionFree(x_new,y_new,near_point.x,near_point.y) and \
                    self.Cost(new_i)+self.c(near_point.x,near_point.y,x_new,y_new)<self.Cost(i):
                        parent_i = self.Tree.getNode(i).getParentIndex()
                        self.Tree.getNode(parent_i).deleteChildrenIndex(i)
                        self.Tree.getNode(i).modifyParentIndex(new_i)
                        self.Tree.getNode(new_i).addChildrenIndex(i)
                goal_reached = self.GoalReached(x_goal, y_goal, x_new, y_new)
                counter += 1

        if counter >= self.max_search_iter:
            rospy.logerr("RRT MAX SEARCH ITERATION REACHED!")
            return None, "Max Iter"
        path = []
        if goal_reached:
            index = new_i
            while index!=0:
                current_node = self.Tree.getNode(index)
                path.append(current_node.point)
                index = current_node.getParentIndex()
            current_node = self.Tree.getNode(index)
            path.append(current_node.point)
            return path[::-1], "found"
        else:
            rospy.logerr("RRT No Path Found!")
            return None, "not found"
        

# test program

x_goal = 0
y_goal = 0
plan = False
def recordPoint(msg):
    global x_goal
    global y_goal
    global plan
    x_goal = msg.point.x
    y_goal = msg.point.y
    plan = True

if __name__=="__main__":
    rospy.init_node("RRT_test")
    rate = rospy.Rate(3000)
    clicked_point = rospy.Subscriber("/clicked_point", PointStamped, recordPoint, queue_size=10 )

    rrt = RRT_star()

    pub = rospy.Publisher("tree_marker", Marker, queue_size=10)
    path_pub = rospy.Publisher("path_marker", Marker, queue_size=10)


    # Marker params
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "marker_tree"
    marker.id = 0
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD
    marker.scale.x = 0.05
    marker.scale.y = 0.05
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    marker.lifetime = rospy.Duration()

    # Path marker
    path_marker = Marker()
    path_marker.header.frame_id = "map"
    path_marker.header.stamp = rospy.Time.now()
    path_marker.ns = "marker_path" 
    path_marker.id = 1
    path_marker.type = Marker.LINE_STRIP
    path_marker.action = Marker.ADD
    path_marker.scale.x = 0.2
    path_marker.scale.y = 0.2
    path_marker.color.r = 1.0
    path_marker.color.g = 0.0
    path_marker.color.b = 0.0
    path_marker.color.a = 1.0
    path_marker.lifetime = rospy.Duration()

    while not rospy.is_shutdown():
        if plan == True:
            if rrt.ObstacleDetection(x_goal, y_goal):
                rospy.logerr("RRT Goal In Obstacle Region!")
            rrt.InitTree()
            path_marker.points = []
            path_pub.publish(path_marker)
            goal_reached = False
            counter = 0
            while (not rospy.is_shutdown()) and counter < rrt.max_search_iter and goal_reached == False:
                x, y = rrt.RandConfAtGoalGaussian(x_goal, y_goal)
                # x, y = rrt.RandConfCircle()
                new_point = Point(x,y,0)
                nearest_i = rrt.NearestNeighbor(x,y)
                nearest_point = rrt.Tree.getNode(nearest_i).point
                x_new, y_new = rrt.Step(nearest_i, x, y)
                if rrt.CollisionFree(nearest_point.x, nearest_point.y, x_new, y_new):
                    neighbor_index_list = rrt.Neighbors(x_new, y_new)
                    # new_i = rrt.Tree.AddNode(Point(x_new, y_new, 0), nearest_i)
                    min_i = nearest_i
                    min_cost = rrt.Cost(nearest_i)+rrt.c(nearest_point.x,nearest_point.y,x_new,y_new)

                    for i in neighbor_index_list:
                        near_point = rrt.Tree.getNode(i).point
                        if rrt.CollisionFree(near_point.x,near_point.y,x_new,y_new) and \
                        rrt.Cost(i)+rrt.c(near_point.x,near_point.y,x_new,y_new)<min_cost:
                            min_i = i
                            min_cost = rrt.Cost(i)+rrt.c(near_point.x,near_point.y,x_new,y_new)

                    # parent_index = rrt.Tree.getNode(new_i).getParentIndex()
                    # rrt.Tree.getNode(parent_index).deleteChildrenIndex(new_i)
                    # rrt.Tree.getNode(new_i).modifyParentIndex(min_i)
                    # rrt.Tree.getNode(min_i).addChildrenIndex(new_i)

                    new_i = rrt.Tree.AddNode(Point(x_new, y_new, 0), min_i)
                    for i in neighbor_index_list:
                        near_point = rrt.Tree.getNode(i).point
                        if rrt.CollisionFree(x_new,y_new,near_point.x,near_point.y) and \
                        rrt.Cost(new_i)+rrt.c(near_point.x,near_point.y,x_new,y_new)<rrt.Cost(i):
                            parent_i = rrt.Tree.getNode(i).getParentIndex()
                            rrt.Tree.getNode(parent_i).deleteChildrenIndex(i)
                            rrt.Tree.getNode(i).modifyParentIndex(new_i)
                            rrt.Tree.getNode(new_i).addChildrenIndex(i)
                    
                    # Print Tree
                    LineList = rrt.Tree.TraverseNodeAddToLineList(0,[])
                    marker.points = LineList
                    pub.publish(marker)
                    rate.sleep()

                    goal_reached = rrt.GoalReached(x_goal, y_goal, x_new, y_new)

                    counter += 1
                    rate.sleep()
            # if goal_reached:
            #     index = new_i
            #     while index!=0:
            #         current_node = rrt.Tree.getNode(index)
            #         path.points.append(current_node.point)
            #         index = current_node.getParentIndex()
            #     current_node = rrt.Tree.getNode(index)
            #     path.points.append(current_node.point)
            #     path_pub.publish(path)
            plan = False
            if counter >= rrt.max_search_iter:
                rospy.logerr("RRT MAX SEARCH ITERATION REACHED!")
            path = []
            if goal_reached:
                index = new_i
                while index!=0:
                    current_node = rrt.Tree.getNode(index)
                    path.append(current_node.point)
                    index = current_node.getParentIndex()
                current_node = rrt.Tree.getNode(index)
                path.append(current_node.point)
            else:
                rospy.logerr("RRT No Path Found!")
            path_marker.points = path
            path_pub.publish(path_marker)
        rate.sleep()
        



    rospy.spin()
