#!/usr/bin/env python3

import rospy
from RRT_star import RRT_Star
from geometry_msgs.msg import Point
from bot_sim.srv import RRTStar, RRTStarResponse

folder_path = "/home/sentry_train_test/AstarTraining/sim_nav/src/bot_sim/scripts/"
node_name="RRT_star_srv"
map_name=rospy.get_param(node_name+'/map_name')
rrt = RRT_Star(folder_path + "DWA/CostMap/"+map_name)

def handle_rrt_star(req):
    global rrt
    rospy.loginfo("Get Client Request: %d, %d", req.curr_x, req.curr_y)
    path, mesg = rrt.rrt_plan_selection(Point(req.curr_x,req.curr_y,0),
                                        Point(req.goal_x,req.goal_y,0))
    if mesg == "found":
        mesg = 1
    elif mesg == "Max Iter":
        mesg = 2
    elif mesg == "Goal in Obstacle":
        mesg = 3
    elif mesg == "not found":
        mesg = 4
    else:
        mesg = 5
    return RRTStarResponse(success=mesg, path=path)

def rrt_star_server():
    rospy.init_node('rrt_star_server')
    s = rospy.Service('rrt_star', RRTStar, handle_rrt_star)
    rospy.spin()
    

if __name__ == "__main__":
    print("RRT server started")
    rrt_star_server()