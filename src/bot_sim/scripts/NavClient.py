#!/usr/bin/env python3

import rospy
import actionlib
from bot_sim.msg import NavActionAction, NavActionGoal, NavActionFeedback, NavActionResult, NavActionFeedback
import sys
import os

script_path = os.path.abspath(os.path.join(os.path.dirname(__file__),"DWA"))
if script_path not in sys.path:
    sys.path.append(script_path)
rospy.loginfo("Added path: %s", script_path)
from test_util_2 import *

def feedback_cb(feedback):
    print('[Feedback] Progress: {0}'.format(feedback.progress_bar))

if __name__ == '__main__':
    rospy.init_node('example_action_client')

    clicked_point_sub = ClickedPointSubscriber()
    action_started = False
    client = actionlib.SimpleActionClient('nav_ctrl', NavActionAction)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if clicked_point_sub.get_clicked():
            clicked_point_sub.set_clicked_false()
            if action_started == True:
                action_started = False
                client.cancel_goal()
            if action_started == False:
                action_started = True
                client.wait_for_server()
                goal = NavActionGoal(clicked_point_sub.clicked_point.x,
                                     clicked_point_sub.clicked_point.y)
                client.send_goal(goal, feedback_cb=feedback_cb)
        rate.sleep()
    if action_started == True:
        client.cancel_goal()

