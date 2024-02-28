#!/usr/bin/env python3
"""
D* Lite grid planning
author: vss2sn (28676655+vss2sn@users.noreply.github.com)
Link to papers:
D* Lite (Link: http://idm-lab.org/bib/abstracts/papers/aaai02b.pd)
Improved Fast Replanning for Robot Navigation in Unknown Terrain
(Link: http://www.cs.cmu.edu/~maxim/files/dlite_icra02.pdf)
Implemented maintaining similarity with the pseudocode for understanding.
Code can be significantly optimized by using a priority queue for U, etc.
Avoiding additional imports based on repository philosophy.
"""
import time
import rospy
import math
import matplotlib.pyplot as plt
import random
from typing import Tuple
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import Odometry
from dstarlite_util import *
show_animation = True
pause_time = 0.001
p_create_random_obstacle = 0
node=0
node_mx=0
class splay_tree_node:
    def __init__(self, key, parent=None):
        self.key = key
        self.parent = parent
        self.son = [None, None]
    def __eq__(self, other):
        if((self is None) or (other is None)):
            return self is other
        if(self.key[1][0]==other.key[1][0]) and \
            (self.key[1][1]==other.key[1][1]) and \
            (self.key[0].x==other.key[0].x) and \
            (self.key[0].y==other.key[0].y):
                return True
        return False
    def __ne__(self, other):
        return not self.__eq__(other)
    def __lt__(self, other):
        if(self.key[1][0]==other.key[1][0]):
            if(self.key[1][1]==other.key[1][1]):
                if(self.key[0].x==other.key[0].x):
                    return self.key[0].y<other.key[0].y
                return self.key[0].x<other.key[0].x
            return self.key[1][1]<other.key[1][1]
        return self.key[1][0]<other.key[1][0]
    def get_s(self):
        return self.parent.son[1]==self
    def rotate(self):
        p=self.parent
        g=p.parent
        x=self.get_s()
        if(g is not None):
            g.son[p.get_s()]=self
        self.parent=g
        p.son[x]=self.son[1-x]
        if(p.son[x] is not None):
            p.son[x].parent=p
        self.son[1-x]=p
        p.parent=self
    def get_min(self):
        mi=self
        if(self.son[0] is not None):
            if(self.son[0]<mi):
                mi=self.son[0].get_min()
        if(self.son[1] is not None):
            if(self.son[1]<mi):
                mi=self.son[1].get_min()
        return mi
class splay:
    def __init__(self):
        self.root = None
    def splay(self, x, goal):
        while(x.parent!=goal):
            p=x.parent
            g=p.parent
            if(g!=goal):
                if(x.get_s()==p.get_s()):
                    p.rotate()
                else:
                    x.rotate()
            x.rotate()
        if(goal  is None):
            self.root=x
    def append(self, key):
        global node
        global node_mx
        node+=1
        node_mx=max(node_mx,node)
        nx=splay_tree_node(key)
        if(self.root is None):
            self.root=nx
            return
        x=self.root
        while(x.son[bool(nx<x)] is not None):
            x=x.son[bool(nx<x)]
        nx.parent=x
        x.son[bool(nx<x)]=nx
        self.splay(nx, None)
    def find(self, key):
        # print("find_start")
        x=self.root
        nx=splay_tree_node(key)
        while(x is not None):
            # print(x.key[1],nx.key[1])
            if(x==nx):
                self.splay(x, None)
                return x
            x=x.son[bool(nx<x)]
        return None
    def delete(self, key):
        global node
        node-=1
        x=self.find(key)
        if(x is None):
            return
        self.splay(x, None)
        if(x.son[0] is None):
            self.root=x.son[1]
            if(self.root is not None):
                self.root.parent=None
        else:
            y=x.son[0]
            while(y.son[1] is not None):
                y=y.son[1]
            self.splay(y, x)
            self.root=y
            y.parent=None
            y.son[1]=x.son[1]
            if(y.son[1] is not None):
                y.son[1].parent=y
    # def return_min_key(self):
    #     x=self.root
    #     return x.get_min().key
    def return_min_key(self):
        x=self.root
        while(x.son[1] is not None):
            x=x.son[1]
        return x.key
class Node:
    def __init__(self, x: int = 0, y: int = 0, cost: float = 0.0):
        self.x = x
        self.y = y
        self.cost = cost

def add_coordinates(node1: Node, node2: Node):
    new_node = Node()
    new_node.x = node1.x + node2.x
    new_node.y = node1.y + node2.y
    new_node.cost = node1.cost + node2.cost
    return new_node


def compare_coordinates(node1: Node, node2: Node):
    return node1.x == node2.x and node1.y == node2.y


class DStarLite:

    # Please adjust the heuristic function (h) if you change the list of
    # possible motions
    motions = [
        Node(1, 0, 1),
        Node(0, 1, 1),
        Node(-1, 0, 1),
        Node(0, -1, 1),
        Node(1, 1, math.sqrt(2)),
        Node(1, -1, math.sqrt(2)),
        Node(-1, 1, math.sqrt(2)),
        Node(-1, -1, math.sqrt(2))
    ]
    costs=[[0, 1, 1], [1, math.sqrt(2), math.sqrt(2)], [1, math.sqrt(2), math.sqrt(2)]]
    def __init__(self, ox: list, oy: list):
        # Ensure that within the algorithm implementation all node coordinates
        # are indices in the grid and extend
        # from 0 to abs(<axis>_max - <axis>_min)
        self.x_min_world = int(min(ox))
        self.y_min_world = int(min(oy))
        self.x_max = int(abs(max(ox) - self.x_min_world)+1)
        self.y_max = int(abs(max(oy) - self.y_min_world)+1)
        self.obstacles = [Node(x - self.x_min_world, y - self.y_min_world)
                          for x, y in zip(ox, oy)]
        self.obstacles_xy = np.array(
            [[obstacle.x, obstacle.y] for obstacle in self.obstacles]
        )
        self.start = Node(0, 0)
        self.goal = Node(0, 0)
        self.U = splay()  # type: ignore
        self.km = 0.0
        self.kold = 0.0
        self.rhs = self.create_grid(float("inf"))
        self.g = self.create_grid(float("inf"))
        self.kill_list=[[[][:] for _ in range(self.y_max)] for _ in range(self.x_max)]
        self.occupied=self.create_grid(0)
        for i in self.obstacles_xy:
            self.occupied[i[0]][i[1]]=1
        self.detected_obstacles_xy = np.empty((0, 2))
        self.xy = np.empty((0, 2))
        if show_animation:
            self.detected_obstacles_for_plotting_x = list()  # type: ignore
            self.detected_obstacles_for_plotting_y = list()  # type: ignore
        self.initialized = False

    def create_grid(self, val: float):
        return np.full((self.x_max, self.y_max), val)

    def is_obstacle(self, node: Node):
        is_in_obstacles = self.occupied[node.x][node.y]
        is_in_detected_obstacles = False
        # if self.detected_obstacles_xy.shape[0] > 0:
        #     x = np.array([node.x])
        #     y = np.array([node.y])
        #     is_x_equal = self.detected_obstacles_xy[:, 0] == x
        #     is_y_equal = self.detected_obstacles_xy[:, 1] == y
        #     is_in_detected_obstacles = (is_x_equal & is_y_equal).any()

        return is_in_obstacles or is_in_detected_obstacles

    def c(self, node1: Node, node2: Node):
        if self.is_obstacle(node2):
            # Attempting to move from or to an obstacle
            return math.inf
        # new_node = Node(node1.x-node2.x, node1.y-node2.y)
        # detected_motion = list(filter(lambda motion:
        #                               compare_coordinates(motion, new_node),
        #                               self.motions))
        return self.costs[node1.x-node2.x][node1.y-node2.y]

    def h(self, s: Node):
        # Cannot use the 2nd euclidean norm as this might sometimes generate
        # heuristics that overestimate the cost, making them inadmissible,
        # due to rounding errors etc (when combined with calculate_key)
        # To be admissible heuristic should
        # never overestimate the cost of a move
        # hence not using the line below
        # return math.hypot(self.start.x - s.x, self.start.y - s.y)

        # Below is the same as 1; modify if you modify the cost of each move in
        # motion
        # return max(abs(self.start.x - s.x), abs(self.start.y - s.y))
        return 1

    def calculate_key(self, s: Node):
        return (min(self.g[s.x][s.y], self.rhs[s.x][s.y]) + self.h(s)
                + self.km, min(self.g[s.x][s.y], self.rhs[s.x][s.y]))

    def is_valid(self, node: Node):
        if 0 <= node.x < self.x_max and 0 <= node.y < self.y_max:
            return True
        return False

    def get_neighbours(self, u: Node):
        return [add_coordinates(u, motion) for motion in self.motions
                if self.is_valid(add_coordinates(u, motion))]

    def pred(self, u: Node):
        # Grid, so each vertex is connected to the ones around it
        return self.get_neighbours(u)

    def succ(self, u: Node):
        # Grid, so each vertex is connected to the ones around it
        return self.get_neighbours(u)

    def initialize(self, start: Node, goal: Node):
        self.start.x = start.x - self.x_min_world
        self.start.y = start.y - self.y_min_world
        self.goal.x = goal.x - self.x_min_world
        self.goal.y = goal.y - self.y_min_world
        if not self.initialized:
            self.initialized = True
            print('Initializing')
            self.U = splay()  # Would normally be a priority queue
            self.km = 0.0
            self.rhs = self.create_grid(math.inf)
            self.g = self.create_grid(math.inf)
            self.kill_list=[[[][:] for _ in range(self.y_max)] for _ in range(self.x_max)]
            self.rhs[self.goal.x][self.goal.y] = 0
            val = self.calculate_key(self.goal)
            self.U.append((self.goal, val))
            self.kill_list[self.goal.x][self.goal.y].append(val)
            self.detected_obstacles_xy = np.empty((0, 2))

    def update_vertex(self, u: Node, sprime: Node):
        # print("update_vertex_begin")
        # t1=time.time()
        if not compare_coordinates(u, self.goal):
            self.rhs[u.x][u.y] = min(self.rhs[u.x][u.y],self.c(u, sprime) +
                                      self.g[sprime.x][sprime.y])
            # print(len(self.kill_list[u.x][u.y]))
        if len(self.kill_list[u.x][u.y]):
            for i in self.kill_list[u.x][u.y]:
                self.U.delete((u, i))
            self.kill_list[u.x][u.y]=[]
        if self.g[u.x][u.y] != self.rhs[u.x][u.y]:
            val = self.calculate_key(u)
            self.U.append((u, val))
            self.kill_list[u.x][u.y].append(val)
        # print(time.time()-t1)
    def compare_keys(self, key_pair1: Tuple[float, float],
                     key_pair2: Tuple[float, float]):
        return key_pair1[0] < key_pair2[0] or \
               (key_pair1[0] == key_pair2[0] and key_pair1[1] < key_pair2[1])

    def compute_shortest_path(self):
        has_elements = (self.U.root is not None)
        start_key_not_updated = self.compare_keys(
            self.U.return_min_key()[1], self.calculate_key(self.start)
        )
        rhs_not_equal_to_g = self.rhs[self.start.x][self.start.y] != \
            self.g[self.start.x][self.start.y]
        while has_elements and start_key_not_updated or rhs_not_equal_to_g:
            # print(has_elements, start_key_not_updated, rhs_not_equal_to_g)
            # print(self.U.return_min_key()[0].x, self.U.return_min_key()[0].y)
            # print(self.start.x, self.start.y)
            # print(self.goal.x, self.goal.y)
            # print(self.U.return_min_key()[1])
            temp=self.U.return_min_key()
            self.kold = temp[1]
            u = temp[0]
            # print(self.kold)
            self.U.delete(temp)
            if self.compare_keys(self.kold, self.calculate_key(u)):
                val = self.calculate_key(u)
                self.U.append((u, val))
                self.kill_list[u.x][u.y].append(val)
            elif self.g[u.x, u.y] > self.rhs[u.x, u.y]:
                self.g[u.x, u.y] = self.rhs[u.x, u.y]
                for s in self.pred(u):
                    self.update_vertex(s,u)
            else:
                self.g[u.x, u.y] = math.inf
                for s in self.pred(u) + [u]:
                    self.update_vertex(s,u)
            start_key_not_updated = self.compare_keys(
                self.U.return_min_key()[1], self.calculate_key(self.start)
            )
            rhs_not_equal_to_g = self.rhs[self.start.x][self.start.y] != \
                self.g[self.start.x][self.start.y]

    def detect_changes(self):
        changed_vertices = list()
        if len(self.spoofed_obstacles) > 0:
            for spoofed_obstacle in self.spoofed_obstacles[0]:
                if compare_coordinates(spoofed_obstacle, self.start) or \
                   compare_coordinates(spoofed_obstacle, self.goal):
                    continue
                changed_vertices.append(spoofed_obstacle)
                self.detected_obstacles_xy = np.concatenate(
                    (
                        self.detected_obstacles_xy,
                        [[spoofed_obstacle.x, spoofed_obstacle.y]]
                    )
                )
                if show_animation:
                    self.detected_obstacles_for_plotting_x.append(
                        spoofed_obstacle.x + self.x_min_world)
                    self.detected_obstacles_for_plotting_y.append(
                        spoofed_obstacle.y + self.y_min_world)
                    plt.plot(self.detected_obstacles_for_plotting_x,
                             self.detected_obstacles_for_plotting_y, ".k")
                    plt.pause(pause_time)
            self.spoofed_obstacles.pop(0)

        # Allows random generation of obstacles
        random.seed()
        if random.random() > 1 - p_create_random_obstacle:
            x = random.randint(0, self.x_max - 1)
            y = random.randint(0, self.y_max - 1)
            new_obs = Node(x, y)
            if compare_coordinates(new_obs, self.start) or \
               compare_coordinates(new_obs, self.goal):
                return changed_vertices
            changed_vertices.append(Node(x, y))
            self.detected_obstacles_xy = np.concatenate(
                (
                    self.detected_obstacles_xy,
                    [[x, y]]
                )
            )
            if show_animation:
                self.detected_obstacles_for_plotting_x.append(x +
                                                              self.x_min_world)
                self.detected_obstacles_for_plotting_y.append(y +
                                                              self.y_min_world)
                plt.plot(self.detected_obstacles_for_plotting_x,
                         self.detected_obstacles_for_plotting_y, ".k")
                plt.pause(pause_time)
        return changed_vertices

    def compute_current_path(self):
        path = list()
        current_point = Node(self.start.x, self.start.y)
        while not compare_coordinates(current_point, self.goal):
            path.append(current_point)
            current_point = min(self.succ(current_point),
                                key=lambda sprime:
                                self.c(current_point, sprime) +
                                self.g[sprime.x][sprime.y])
        path.append(self.goal)
        return path

    def compare_paths(self, path1: list, path2: list):
        if len(path1) != len(path2):
            return False
        for node1, node2 in zip(path1, path2):
            if not compare_coordinates(node1, node2):
                return False
        return True

    def display_path(self, path: list, colour: str, alpha: float = 1.0):
        px = [(node.x + self.x_min_world) for node in path]
        py = [(node.y + self.y_min_world) for node in path]
        drawing = plt.plot(px, py, colour, alpha=alpha)
        plt.pause(pause_time)
        return drawing

    def main(self, start: Node, goal: Node,
             spoofed_ox: list, spoofed_oy: list):
        self.spoofed_obstacles = [[Node(x - self.x_min_world,
                                        y - self.y_min_world)
                                   for x, y in zip(rowx, rowy)]
                                  for rowx, rowy in zip(spoofed_ox, spoofed_oy)
                                  ]
        pathx = []
        pathy = []
        self.initialize(start, goal)
        last = self.start
        print("Computing shortest path")
        t1=time.time()
        import cProfile
        cProfile.runctx('self.compute_shortest_path()', globals(), locals())
        # self.compute_shortest_path()
        print(time.time()-t1)
        pathx.append(self.start.x + self.x_min_world)
        pathy.append(self.start.y + self.y_min_world)

        if show_animation:
            current_path = self.compute_current_path()
            previous_path = current_path.copy()
            previous_path_image = self.display_path(previous_path, ".c",
                                                    alpha=0.3)
            current_path_image = self.display_path(current_path, ".c")

        while not compare_coordinates(self.goal, self.start):
            if self.g[self.start.x][self.start.y] == math.inf:
                print("No path possible")
                return False, pathx, pathy
            self.start = min(self.succ(self.start),
                             key=lambda sprime:
                             self.c(self.start, sprime) +
                             self.g[sprime.x][sprime.y])
            pathx.append(self.start.x + self.x_min_world)
            pathy.append(self.start.y + self.y_min_world)
            if show_animation:
                current_path.pop(0)
                plt.plot(pathx, pathy, "-r")
                plt.pause(pause_time)
            changed_vertices = self.detect_changes()
            if len(changed_vertices) != 0:
                print("New obstacle detected")
                self.km += self.h(last)
                last = self.start
                for u in changed_vertices:
                    if compare_coordinates(u, self.start):
                        continue
                    self.rhs[u.x][u.y] = math.inf
                    self.g[u.x][u.y] = math.inf
                    self.update_vertex(u)
                self.compute_shortest_path()

                if show_animation:
                    new_path = self.compute_current_path()
                    if not self.compare_paths(current_path, new_path):
                        current_path_image[0].remove()
                        previous_path_image[0].remove()
                        previous_path = current_path.copy()
                        current_path = new_path.copy()
                        previous_path_image = self.display_path(previous_path,
                                                                ".c",
                                                                alpha=0.3)
                        current_path_image = self.display_path(current_path,
                                                               ".c")
                        plt.pause(pause_time)
        print("Path found")
        return True, pathx, pathy
def solver(msg):
    gx=msg.point.x
    gy=msg.point.y
    map_util=MapUtil(cost_map_file=' ')
    map = map_util.get_map()
    map_info = map_util.get_map_info()
    print(msg,map_info)
    robot_msg=rospy.wait_for_message("/odom",Odometry)
    origin_x = map_info.origin.position.x
    origin_y = map_info.origin.position.y
    sx=robot_msg.pose.pose.position.x
    sy=robot_msg.pose.pose.position.y
    print(sx,sy)
    sx, sy = map_util.act_pos_to_grid_pos(sx, sy)
    gx, gy = map_util.act_pos_to_grid_pos(gx, gy)
    width = map_info.width
    height = map_info.height
    map_2d = np.array(map).reshape(height, width)
    ox=[]
    oy=[]
    for i in range(len(map_2d)):
        for j in range(len(map_2d[0])):
            if map_2d[i][j] > 10:
                ox.append(j)
                oy.append(i)
    if show_animation:
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "og")
        plt.plot(gx, gy, "xb")
        plt.grid(True)
        plt.axis("equal")
        label_column = ['Start', 'Goal', 'Path taken',
                        'Current computed path', 'Previous computed path',
                        'Obstacles']
        columns = [plt.plot([], [], symbol, color=colour, alpha=alpha)[0]
                   for symbol, colour, alpha in [['o', 'g', 1],
                                                 ['x', 'b', 1],
                                                 ['-', 'r', 1],
                                                 ['.', 'c', 1],
                                                 ['.', 'c', 0.3],
                                                 ['.', 'k', 1]]]
        plt.legend(columns, label_column, bbox_to_anchor=(1, 1), title="Key:",
                   fontsize="xx-small")
        plt.plot()
        plt.pause(pause_time)

    # Obstacles discovered at time = row
    # time = 1, obstacles discovered at (0, 2), (9, 2), (4, 0)
    # time = 2, obstacles discovered at (0, 1), (7, 7)
    # ...
    # when the spoofed obstacles are:
    # spoofed_ox = [[0, 9, 4], [0, 7], [], [], [], [], [], [5]]
    # spoofed_oy = [[2, 2, 0], [1, 7], [], [], [], [], [], [4]]

    # Reroute
    # spoofed_ox = [[], [], [], [], [], [], [], [40 for _ in range(10, 21)]]
    # spoofed_oy = [[], [], [], [], [], [], [], [i for i in range(10, 21)]]

    # Obstacles that demostrate large rerouting
    t1=time.time()
    print("start")
    dstarlite = DStarLite(ox, oy)
    dstarlite.main(Node(x=sx, y=sy), Node(x=gx, y=gy),
                   spoofed_ox=[], spoofed_oy=[])
    t2=time.time()
    print("end")
    print(t2-t1)
    global node_mx
    print(node_mx)
                
def main():
    rospy.init_node('dstarlite', anonymous=True)
    clicked_point=rospy.Subscriber("/clicked_point",PointStamped,solver, queue_size=10)
    rospy.spin()
    # start and goal position
    

if __name__ == "__main__":
    main()
