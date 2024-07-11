#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker
from RRT_star import RRTStar
from rrt_util import *

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
    rate = rospy.Rate(30)
    clicked_point = rospy.Subscriber("/clicked_point", PointStamped, recordPoint, queue_size=10 )

    folder_path = "/home/sentry_train_test/AstarTraining/sim_nav/src/bot_sim/scripts/RRT/"
    file_name = "CostMap/CostMapR0d5"
    rrt = RRTStar(cost_map_path=folder_path+file_name)

    pub = rospy.Publisher("tree_marker", Marker, queue_size=10)
    path_pub = rospy.Publisher("path_marker", Marker, queue_size=10)

    counter = 0

    # Path marker
    path = Marker()
    path.header.frame_id = "map"
    path.header.stamp = rospy.Time.now()
    path.ns = "marker_path"
    path.id = 1
    path.type = Marker.LINE_STRIP
    path.action = Marker.ADD
    path.scale.x = 0.2
    path.scale.y = 0.2
    path.color.r = 1.0
    path.color.g = 0.0
    path.color.b = 0.0
    path.color.a = 1.0
    path.lifetime = rospy.Duration()

    while not rospy.is_shutdown():
        if plan == True:
            planned_path, info = rrt.rrt_plan_selection(Point(0,0,0), Point(x_goal, y_goal, 0.0),10)
            print(info)
            if info == "found":
                path.points = planned_path
                for i in range(len(planned_path)):
                    planned_path[i].z = 0.0
                path_pub.publish(path)
            plan = False
        rate.sleep()

    rospy.spin()

