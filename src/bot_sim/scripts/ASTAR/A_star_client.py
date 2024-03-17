#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import Odometry
import numpy as np
from typing import List, Tuple
import pickle
import math
import matplotlib.pyplot as plt
from astar_util import *
from bot_sim.srv import Astar, AstarResponse
def point_stamp_callback(msg):
    global clicked_point, clicked
    print('Received clicked point')
    clicked_point = msg
    clicked = True
def odom_callback(msg):
    global odom_info, get_odom
    # print('Received odometry')
    odom_info = msg
    get_odom = True
# if __name__ == '__main__':
print("started")
global clicked_point, clicked, odom_info, get_odom
node_name="A_star_client"
rospy.init_node(node_name)
path_planner = rospy.ServiceProxy('A_star_service', Astar)
odom_name = rospy.get_param(node_name+'/odom_name')
clicked_point = PointStamped()
clicked = False
Point_stamp_subscriber=rospy.Subscriber('clicked_point', PointStamped, point_stamp_callback)
odom_info = Odometry()
get_odom = False
odom_subscriber = rospy.Subscriber(odom_name, Odometry, odom_callback)
rate = rospy.Rate(10)
# rospy.spin()
while not rospy.is_shutdown():
    if clicked and get_odom:
        print("start_planning")
        start=Point()
        start.x = odom_info.pose.pose.position.x
        start.y = odom_info.pose.pose.position.y
        goal=Point()
        goal.x = clicked_point.point.x
        goal.y = clicked_point.point.y
        path_planner(start.x, start.y, goal.x, goal.y)
        clicked = False
        get_odom = False
    rate.sleep()
    # print("still_alive")
# print("while_complite")