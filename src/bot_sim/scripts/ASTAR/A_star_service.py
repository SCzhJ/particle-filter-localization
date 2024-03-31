#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PointStamped
import numpy as np
from typing import List, Tuple
import pickle
import math
import matplotlib.pyplot as plt
from astar_util import *
from bot_sim.srv import Astar, AstarResponse
from bot_sim.msg import TwoPoints #记得把bot_sim_laser改成你的package name
show_animation = False

class AStarPlanner:

    def __init__(self, ox, oy, rr):
        """
        Initialize grid map for a star planning

        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        resolution: grid resolution [m]
        rr: robot radius[m]
        """
        print("map_read")
        self.map_util = MapUtil(cost_map_file=' ')
        print("map_read_done")
        self.resolution = self.map_util.grid_info.resolution
        self.rr = rr
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.obstacle_map = None
        self.x_width, self.y_width = 0, 0
        self.motion = self.get_motion_model()
        self.origin_x = self.map_util.origin_x
        self.origin_y = self.map_util.origin_y
        

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy):
        """
        A star path search

        input:
            s_x: start x position [m]
            s_y: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """

        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)
        if(self.father[start_node.x*self.y_width+start_node.y]!=self.father[goal_node.x*self.y_width+goal_node.y]):
            print("No path")
            return [], []
        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        while True:
            if len(open_set) == 0:
                print("Open set is empty..")
                break
            c_id = min(open_set,key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node,open_set[o]))
            current = open_set[c_id]

            # # show graph
            # if show_animation:  # pragma: no cover
            #     plt.plot(self.calc_grid_position(current.x, self.min_x),
            #              self.calc_grid_position(current.y, self.min_y), "xc")
            #     # for stopping simulation with the esc key.
            #     plt.gcf().canvas.mpl_connect('key_release_event',
            #                                  lambda event: [exit(
            #                                      0) if event.key == 'escape' else None])
            #     if len(closed_set.keys()) % 10 == 0:
            #         plt.pause(0.001)

            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)
                n_id = self.calc_grid_index(node)

                # If the node is not safe, do nothing
                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)
        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index

        return rx, ry

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def calc_grid_position(self, index, min_position):
        """
        calc grid position

        :param index:
        :param min_position:
        :return:
        """
        pos = index * self.resolution + min_position
        return pos

    def reverse_calc_grid_position(self, pos, min_position):
        index = (pos - min_position)/self.resolution
        return int(index)

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        elif py < self.min_y:
            return False
        elif px >= self.max_x:
            return False
        elif py >= self.max_y:
            return False

        # collision check
        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    def obstacle_value_detection(self, real_coord_x: float, real_coord_y: float) -> bool:
        """
        Detect obstacle inputting real coordinate.
        if the distance is within certain range, allow stepping over the obstacle
            enlargement area
        """
        grid_x, grid_y = self.map_util.act_pos_to_grid_pos(real_coord_x, real_coord_y)
        occ_val = self.map_util.occupancy_value_check_cost_map_grid_coord(grid_x, grid_y)
        if self.line_cost(self.robot_position.x, self.robot_position.y, real_coord_x, real_coord_y) < self.obstacle_enlargement_tolerance_range:
            if occ_val > 75:
                return True
            else:
                return False
        else:
            if occ_val > 65:
                return True
            else:
                return False

    def obstacle_detection(self, real_coord_x: float, real_coord_y: float) -> bool:
        grid_x, grid_y = self.map_util.act_pos_to_grid_pos(real_coord_x, real_coord_y)
        occ_val = self.map_util.occupancy_value_check_cost_map_grid_coord(grid_x, grid_y)
        if occ_val == -1 or occ_val > 65:
            return True
        return False

    def calc_obstacle_map(self, ox, oy):

        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))
        print("min_x:", self.min_x)
        print("min_y:", self.min_y)
        print("max_x:", self.max_x)
        print("max_y:", self.max_y)

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        print("x_width:", self.x_width)
        print("y_width:", self.y_width)
        print(self)
        rospy.loginfo("x_width: %s, y_width: %s", self.x_width, self.y_width)
        # obstacle map generation
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
                             # 获取当前的仿真时间
        #并查集
        self.father=[i for i in range(self.x_width*self.y_width)]
        sim_time = rospy.Time.now()
        print("Current simulation time: ", sim_time)
        rr=self.rr
        for iox, ioy in zip(ox, oy):
            sx=max(self.reverse_calc_grid_position(iox-rr*2, self.min_x),0)
            ex=min(self.reverse_calc_grid_position(iox+rr*2, self.min_x),self.x_width)
            sy=max(self.reverse_calc_grid_position(ioy-rr*2, self.min_y),0)
            ey=min(self.reverse_calc_grid_position(ioy+rr*2, self.min_y),self.y_width)
            # print(sx,ex,sy,ey)
            for ix in range(sx,ex):
                x = self.calc_grid_position(ix, self.min_x)
                for iy in range(sy,ey):
                    y = self.calc_grid_position(iy, self.min_y)
                    d = math.hypot(iox - x, ioy - y)
                    if d <= rr:
                        self.obstacle_map[ix][iy] = True
        # for ix in range(self.x_width):
        #     x = self.calc_grid_position(ix, self.min_x)
        #     print(x)
        #     for iy in range(self.y_width):
        #         y = self.calc_grid_position(iy, self.min_y)
        #         for iox, ioy in zip(ox, oy):
        #             d = math.hypot(iox - x, ioy - y)
        #             if d <= self.rr:
        #                 self.obstacle_map[ix][iy] = True
        #                 break
        sim_time = rospy.Time.now()
        print("Current simulation time: ", sim_time)

    def get_father(self,x):
        if self.father[x]!=x:
            self.father[x]=self.get_father(self.father[x])
        return self.father[x]

    def merge(self,x,y):
        x=self.get_father(x)
        y=self.get_father(y)
        if x!=y:
            self.father[x]=y
    
    def solve_the_equalvalence_class(self):
        for i in range(self.x_width):
            for j in range(self.y_width):
                if not self.obstacle_map[i][j]:
                    for k in range(8):
                        ni=i+self.motion[k][0]
                        nj=j+self.motion[k][1]
                        if ni>=0 and ni<self.x_width and nj>=0 and nj<self.y_width and not self.obstacle_map[ni][nj]:
                            self.merge(i*self.y_width+j,ni*self.y_width+nj)
        for i in range(self.x_width):
            for j in range(self.y_width):
                self.get_father(i*self.y_width+j)
    
    def get_obstacle_coordinates(self):
        grid_map = self.map_util.get_map()
        grid_info = self.map_util.get_map_info()
        ox, oy = [], []
        for i in range(len(grid_map)):
            if grid_map[i] > 50 or grid_map == -1:  # assuming values > 50 represent obstacles
                x = (i % grid_info.width)*self.resolution
                y = (i // grid_info.width)*self.resolution
                print("obstacle:x: ", x, "y: ", y)
                ox.append(x)
                oy.append(y)
        self.ox = ox
        self.oy = oy

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion
def calculate_angle(x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    return math.atan2(dy, dx)

x_goal = 0
y_goal = 0
plan = False
def response_function(msg):
    print("Planning")
    path_marker.points = []
    # sx, sy, _ = odom_subscriber.get_pose()
    sx=msg.curr_x
    sy=msg.curr_y
    x_goal=msg.goal_x
    y_goal=msg.goal_y
    rospy.loginfo("sx: %s, sy: %s", sx, sy)
    sx = sx - a_star.origin_x
    sy = sy - a_star.origin_y
    x_goal = x_goal - a_star.origin_x
    y_goal = y_goal - a_star.origin_y
    rospy.loginfo("Astar Planning")
    rx, ry = a_star.planning(sx, sy, x_goal, y_goal)
    rospy.loginfo("Astar Planning Done")
    path=[]
    last=Point()
    last.z=-114514
    for i in range(0,len(rx)):
        x=rx[i]
        y=ry[i]
        rospy.loginfo("x: %s, y: %s", x, y)
        p = Point()
        p.x = x + a_star.origin_x
        p.y = y + a_star.origin_y
        if(i!=len(rx)-1):
            p.z = calculate_angle(x, y, rx[i+1], ry[i+1])
        else:
            p.z = -114514
        if(last.z!=p.z):
            path_marker.points.append(p)
            path.append(p)
        last=p
    Pointlist = AstarResponse()
    Pointlist.path = path
    path_pub.publish(path_marker)
    return Pointlist
if __name__ == '__main__':
    try:
        print("start")
        rospy.init_node('a_star_planner')
        print("init_node")
        rate = rospy.Rate(100)

        print("a_star_init")
        a_star = AStarPlanner([], [], 0.3)
        print("a_star_init_done")
        a_star.get_obstacle_coordinates()
        a_star.calc_obstacle_map(a_star.ox, a_star.oy)
        a_star.solve_the_equalvalence_class()
        path_pub = rospy.Publisher("path_marker", Marker, queue_size=10)
        print("init_started")
        print("init_done")
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
        service = rospy.Service('A_star_service', Astar, response_function) # 创建服务，当有请求到'my_service'时，调用handle_request函数
        rospy.spin()
    except KeyboardInterrupt:
        pass
