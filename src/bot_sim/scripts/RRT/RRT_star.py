#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PointStamped
import numpy as np
from rrt_util import TreeNode, MapUtil, TreeUtil
from typing import List, Tuple
import pickle

class RRTStar:
    def __init__(self, cost_map_path: str, q_init: Point = Point(0,0,0)):
        """Initialize the RRTStar class."""
        self.map_util = MapUtil(cost_map_file=cost_map_path)
        self.map_util.load_cost_map(cost_map_file=cost_map_path)
        self.sampling_origin = [-9, -9]
        self.sampling_range  = [19, 18]
        self.sampling_radius = 30
        self.normal_sigma = 0.7
        self.normal_enlargement = 15
        self.max_iter = 10000
        self.step_size = 0.5
        self.gamma     = 4
        self.eta       = 0.4 * self.gamma
        self.d         = 2
        self.default_goal_radius = 1.0
        self.goal_radius = self.default_goal_radius
        self.max_search_iter = 1000
        self.coll_step= 0.1
        self.tree = TreeUtil(q_init)
    
    def set_goal_radius(self, r: float):
        """Set the goal radius."""
        self.goal_radius = r
    
    def rand_conf_at_goal_gaussian(self, goal_x: float, goal_y: float) -> Tuple[float, float]:
        """Generate a random configuration at the goal using a Gaussian distribution."""
        rand_range = np.random.normal(self.normal_sigma) * self.normal_enlargement
        rand_theta = np.random.rand() * 2 * np.pi
        rand_x = np.cos(rand_theta) * rand_range + goal_x
        rand_y = np.sin(rand_theta) * rand_range + goal_y
        return rand_x, rand_y
    
    def rand_conf_circle(self) -> Tuple[float, float]:
        """Generate a random configuration in a circle."""
        rand_range = np.random.rand() * self.sampling_radius
        rand_theta = np.random.rand() * 2 * np.pi
        rand_x = np.cos(rand_theta) * rand_range
        rand_y = np.sin(rand_theta) * rand_range
        return rand_x, rand_y

    def rand_conf_rect(self) -> Tuple[float, float]:
        """Generate a random configuration in a rectangle."""
        rand_x = self.sampling_origin[0] + np.random.rand() * self.sampling_range[0]
        rand_y = self.sampling_origin[1] + np.random.rand() * self.sampling_range[1]
        return rand_x, rand_y

    def rand_conf_rec_rej(self) -> Tuple[float, float]:
        """Generate a random configuration in a rectangle, rejecting the samples at obstacle."""
        done = False
        counter = 0
        while (not done) and counter != self.max_iter:
            x, y = self.rand_conf_rect()
            if not self.obstacle_detection(x, y):
                done = True
            counter += 1
        if counter == self.max_iter:
            rospy.logerr("Sampling iteration reached maximum!")
        return x, y

    def obstacle_detection(self, real_coord_x: float, real_coord_y: float) -> bool:
        """Detect obstacle inputting real coordinate."""
        grid_x, grid_y = self.map_util.act_pos_to_grid_pos(real_coord_x, real_coord_y)
        return self.map_util.occupancy_check_cost_map_grid_coord(grid_x, grid_y)

    def collision_free(self, x: float, y: float, x_d: float, y_d: float) -> bool:
        """Detect if collision exists between points."""
        x_del = x_d - x
        y_del = y_d - y
        vec_len = np.sqrt(x_del**2 + y_del**2)
        x_del = self.coll_step * self.step_size * x_del / vec_len
        y_del = self.coll_step * self.step_size * y_del / vec_len
        length = 0
        while length < self.step_size:
            x += x_del
            y += y_del
            if self.obstacle_detection(x, y):
                return False
            length += self.coll_step * self.step_size
        return True

    def nearest_neighbor(self, x: float, y: float) -> int:
        """Find the nearest neighbor."""
        nearest_dist  = 100000
        nearest_index = -1
        for i in range(len(self.tree.tree)):
            point = self.tree.get_node(i).point
            new_dist = np.sqrt((point.x - x)**2 + (point.y - y)**2)
            if new_dist < nearest_dist:
               nearest_index = i
               nearest_dist = new_dist
        return nearest_index

    def neighbors(self, x: float, y: float) -> List[int]:
        """Return a list of neighboring indices."""
        card_v = len(self.tree.tree)
        radius = min(self.gamma * (np.log(card_v) / card_v) ** (1 / self.d), self.eta)
        neighbor_list = []
        for i in range(len(self.tree.tree)):
            point = self.tree.get_node(i).point
            dist = np.sqrt((x - point.x) ** 2 + (y - point.y) ** 2)
            if dist < radius:
                neighbor_list.append(i)
        return neighbor_list

    def line_cost(self, x_1: float, y_1: float, x_2: float, y_2: float) -> float:
        """Calculate the cost of a line."""
        return np.sqrt((x_1 - x_2) ** 2 + (y_1 - y_2) ** 2)

    def path_cost(self, node_i: int) -> float:
        """Calculate the cost of a path to a point."""
        cumulative_cost = 0
        current_node = self.tree.get_node(node_i)
        parent_index = current_node.get_parent_index()
        while parent_index != -1: 
            parent_node = self.tree.get_node(parent_index)
            cumulative_cost += self.line_cost(current_node.point.x,
                                              current_node.point.y,
                                              parent_node.point.x,
                                              parent_node.point.y)
            current_node = parent_node
            parent_index = current_node.get_parent_index()
        return cumulative_cost
    
    def step(self, nearest_index: int, new_x: float, new_y: float) -> Tuple[float, float]:
        """Take a step from the nearest point towards the new point."""
        nearest_point = self.tree.get_node(nearest_index).point
        x_del = new_x - nearest_point.x
        y_del = new_y - nearest_point.y
        vec_len = np.sqrt(x_del**2 + y_del**2)
        if vec_len == 0:
            rospy.logerr("vec_len is zero!")
            return None, None
        x_del = self.step_size * x_del / vec_len
        y_del = self.step_size * y_del / vec_len

        x_new = nearest_point.x + x_del
        y_new = nearest_point.y + y_del

        return x_new, y_new

    def goal_reached(self, goal_x: float, goal_y: float, new_x: float, new_y: float) -> bool:
        """Check if the goal has been reached."""
        if self.line_cost(goal_x, goal_y, new_x, new_y) < self.goal_radius:
            return True
        else:
            return False

    def find_nearest(self, x: float, y: float) -> Tuple[float, float, int]:
        """Find the nearest point in the tree to the given coordinates."""
        nearest_i = self.nearest_neighbor(x, y)
        nearest_point = self.tree.get_node(nearest_i).point
        return nearest_point.x, nearest_point.y, nearest_i
    
    def find_min_cost(self, x_new: float, y_new: float, nearest_i: int, nearest_x: float, nearest_y: float) -> Tuple[int, List[int]]:
        """Find the minimum cost to reach the new point."""
        neighbor_index_list = self.neighbors(x_new, y_new)
        min_i = nearest_i
        min_cost = self.path_cost(nearest_i) + self.line_cost(nearest_x, nearest_y, x_new, y_new)
        for i in neighbor_index_list:
            near_point = self.tree.get_node(i).point
            if self.collision_free(near_point.x, near_point.y, x_new, y_new) and \
            self.path_cost(i) + self.line_cost(near_point.x, near_point.y, x_new, y_new) < min_cost:
                min_i = i
                min_cost = self.path_cost(i) + self.line_cost(near_point.x, near_point.y, x_new, y_new)
        return min_i, neighbor_index_list
    
    def update_tree(self, x_new: float, y_new: float, min_i: int, neighbor_index_list: List[int]) -> int:
        """Update the tree with the new point."""
        new_i = self.tree.add_node(Point(x_new, y_new, 0), min_i)
        for i in neighbor_index_list:
            near_point = self.tree.get_node(i).point
            if self.collision_free(x_new, y_new, near_point.x, near_point.y) and \
            self.path_cost(new_i) + self.line_cost(near_point.x, near_point.y, x_new, y_new) < self.path_cost(i):
                parent_i = self.tree.get_node(i).get_parent_index()
                self.tree.get_node(parent_i).delete_children_index(i)
                self.tree.get_node(i).modify_parent_index(new_i)
                self.tree.get_node(new_i).add_children_index(i)
        return new_i
    
    def build_path(self, new_i: int) -> List[Point]:
        """Build the path from the start to the new point."""
        path = []
        index = new_i
        while index != 0:
            current_node = self.tree.get_node(index)
            path.append(current_node.point)
            index = current_node.get_parent_index()
        current_node = self.tree.get_node(index)
        path.append(current_node.point)
        return path[::-1]

    def rrt_plan(self, robot_point: Point, goal_point: Point) -> Tuple[List[Point], str]:
        """Plan a path using RRT*."""
        self.tree = TreeUtil(robot_point)
        x_goal = goal_point.x
        y_goal = goal_point.y
        rospy.loginfo("RRT Planning")
        if self.obstacle_detection(x_goal, y_goal):
            rospy.logerr("RRT Goal In Obstacle Region!")
            return None, "Goal in Obstacle"
        goal_reached = False
        counter = 0
        while counter < self.max_search_iter and (not goal_reached) and (not rospy.is_shutdown()):
            x, y = self.rand_conf_at_goal_gaussian(x_goal, y_goal)
            nearest_x, nearest_y, nearest_i = self.find_nearest(x, y)
            x_new, y_new = self.step(nearest_i, x, y)
            if self.collision_free(nearest_x, nearest_y, x_new, y_new):
                min_i, neighbor_index_list = self.find_min_cost(x_new, y_new, nearest_i, nearest_x, nearest_y)
                new_i = self.update_tree(x_new, y_new, min_i, neighbor_index_list)
                goal_reached = self.goal_reached(x_goal, y_goal, x_new, y_new)
                counter += 1
        if counter >= self.max_search_iter:
            rospy.logerr("RRT MAX SEARCH ITERATION REACHED!")
            return None, "Max Iter"
        if goal_reached:
            return self.build_path(new_i), "found"
        else:
            rospy.logerr("RRT No Path Found!")
            return None, "not found"
        
    def rrt_plan_selection(self, robot_pose: Point, goal_point: Point,iteration: int=5) -> Tuple[List[Point], str]:
        """Plan a path using RRT* and select the optimal one."""
        min_cost = 10000
        optimal_path = []
        for _ in range(iteration):
            path, info = self.rrt_plan(robot_pose, goal_point)
            if info != "found":
                rospy.logerr("RRT ERROR!")
                return [], "error!"
            cost = 0
            prev = path[0]
            for i in range(1, len(path)):
                curr = path[i]
                cost += self.line_cost(prev.x, prev.y, curr.x, curr.y)
                prev = curr
            if cost < min_cost:
                optimal_path = path
                min_cost = cost
            for i in range(len(optimal_path)-1):
                this_point = optimal_path[i]
                next_point = optimal_path[i+1]
                dx = next_point.x - this_point.x
                dy = next_point.y - this_point.y
                theta = np.arctan2(dy, dx)
                optimal_path[i].z = theta
            optimal_path[-1].z = optimal_path[-2].z
        return optimal_path, "found"

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
    rate = rospy.Rate(100)
    clicked_point = rospy.Subscriber("/clicked_point", PointStamped, recordPoint, queue_size=10 )

    folder_path = "/home/sentry_train_test/AstarTraining/sim_nav/src/bot_sim/scripts/RRT/"
    file_path = "CostMap/CostMapR0d5"
    rrt = RRTStar(folder_path+file_path)

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
            """Plan a path using RRT*."""
            rospy.loginfo("RRT Planning")
            if rrt.obstacle_detection(x_goal, y_goal):
                rospy.logerr("RRT Goal In Obstacle Region!")
            else:
                goal_reached = False
                counter = 0
                rrt.tree = TreeUtil(Point(0,0,0))
                while counter < rrt.max_search_iter and (not goal_reached) and (not rospy.is_shutdown()):
                    x, y = rrt.rand_conf_at_goal_gaussian(x_goal, y_goal)
                    nearest_x, nearest_y, nearest_i = rrt.find_nearest(x, y)
                    x_new, y_new = rrt.step(nearest_i, x, y)
                    if rrt.collision_free(nearest_x, nearest_y, x_new, y_new):
                        min_i, neighbor_index_list = rrt.find_min_cost(x_new, y_new, nearest_i, nearest_x, nearest_y)
                        new_i = rrt.update_tree(x_new, y_new, min_i, neighbor_index_list)
                        goal_reached = rrt.goal_reached(x_goal, y_goal, x_new, y_new)
                        counter += 1
                    # Print Tree
                    LineList = rrt.tree.traverse_node_add_to_line_list(0,[])
                    marker.points = LineList
                    pub.publish(marker)
                    rate.sleep()
                if counter >= rrt.max_search_iter:
                    rospy.logerr("RRT MAX SEARCH ITERATION REACHED!")
                if goal_reached:
                    path_marker.points = rrt.build_path(new_i)[::-1]
                    path_pub.publish(path_marker)
                else:
                    rospy.logerr("RRT No Path Found!")
            plan = False
        rate.sleep()
        
    rospy.spin()
