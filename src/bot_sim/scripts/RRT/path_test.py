#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PointStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from visualization_msgs.msg import Marker, MarkerArray

from geometry_msgs.msg import Point

from RRT import RRT
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
            planned_path, info = rrt.rrt_plan(x_goal,y_goal)
            print(info)
            if info == "found":
                path.points = planned_path
                path_pub.publish(path)
            plan = False
        rate.sleep()

    rospy.spin()


# if __name__=="__main__":
#     rospy.init_node("test_p")
#     rate = rospy.Rate(5)
#     rrt = RRT()

#     pub = rospy.Publisher("tree_markers", Marker, queue_size=10)

#     counter = 0
#     marker = Marker()
#     marker.header.frame_id = "map"
#     marker.header.stamp = rospy.Time.now()
#     marker.ns = "marker_tree"
    
#     marker.id = 0
#     marker.type = Marker.LINE_LIST
#     marker.action = Marker.ADD
#     marker.scale.x = 0.1
#     marker.scale.y = 0.1
#     marker.color.r = 0.0
#     marker.color.g = 1.0
#     marker.color.b = 0.0
#     marker.color.a = 1.0
#     marker.lifetime = rospy.Duration()

#     while (not rospy.is_shutdown()) and counter<20:
#         x, y = rrt.RandConfRej()
#         new_point = Point(x,y,0)
#         nearest_i = rrt.NearestNeighbor(x,y)
#         rrt.Tree.AddNode(new_point,nearest_i)
#         LineList = rrt.Tree.TraverseNodeAddToLineList(0,[])
#         marker.points = LineList

#         pub.publish(marker)
#         rate.sleep()

#     rospy.spin()



# x = 0
# y = 0
# def recordPoint(msg):
    # global x
    # global y
    # x = msg.point.x
    # y = msg.point.y

# if __name__ == '__main__':
    # rospy.init_node('marker_tree_publisher', anonymous=True)
    # rate = rospy.Rate(10)
    # rrt = RRT()
    # clicked_point = rospy.Subscriber("/clicked_point", PointStamped, recordPoint, queue_size=10 )
    
    # while not rospy.is_shutdown():
        # print(rrt.ObstacleDetection(x,y))
        # rate.sleep()

# def publish_marker_tree():
    # rospy.init_node('marker_tree_publisher', anonymous=True)
    # marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    # rate = rospy.Rate(10)  # 10 Hz

    # while not rospy.is_shutdown():
        # marker_id = 0

        # # Create the root point
        # marker = Marker()
        # marker.header.frame_id = "map"
        # marker.header.stamp = rospy.Time.now()
        # marker.ns = "marker_tree"
        
        # marker.id = marker_id
        # marker_id += 1

        # marker.type = Marker.LINE_LIST
        # marker.action = Marker.ADD
        # marker.scale.x = 0.1
        # marker.scale.y = 0.1
        # marker.color.r = 0.0
        # marker.color.g = 1.0
        # marker.color.b = 0.0
        # marker.color.a = 1.0
        # marker.lifetime = rospy.Duration()

        # root_point = Point()
        # root_point.x = 0.0
        # root_point.y = 0.0
        # root_point.z = 0.0

        # point_1 = Point()
        # point_1.x = -9
        # point_1.y = -9
        # point_1.z = 1.0


        # point_2 = Point()
        # point_2.x = -9
        # point_2.y = 9
        # point_2.z = 1.0

        # point_3 = Point()
        # point_3.x = 10
        # point_3.y = 9
        # point_3.z = 1.0

        # point_4 = Point()
        # point_4.x = 10
        # point_4.y = -9
        # point_4.z = 1.0


        # marker.points.append(point_1)
        # marker.points.append(point_2)
        # marker.points.append(point_1)
        # marker.points.append(point_4)
        # marker.points.append(point_3)
        # marker.points.append(point_2)
        # marker.points.append(point_3)
        # marker.points.append(point_4)

        # marker_pub.publish(marker)
        # rate.sleep()

# if __name__ == '__main__':
    # try:
        # publish_marker_tree()
    # except rospy.ROSInterruptException:
        # pass

# def publish_marker():
    # rospy.init_node('marker_publisher', anonymous=True)
    # marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    # rate = rospy.Rate(10)  # 10 Hz

    # while not rospy.is_shutdown():
        # marker = Marker()
        # marker.header.frame_id = "base_link"
        # marker.header.stamp = rospy.Time.now()
        # marker.ns = "basic_shapes"
        # marker.id = 0
        # marker.type = Marker.SPHERE
        # marker.action = Marker.ADD
        # marker.pose.position.x = 1.0
        # marker.pose.position.y = 2.0
        # marker.pose.position.z = 0.0
        # marker.pose.orientation.x = 0.0
        # marker.pose.orientation.y = 0.0
        # marker.pose.orientation.z = 0.0
        # marker.pose.orientation.w = 1.0
        # marker.scale.x = 0.2
        # marker.scale.y = 0.2
        # marker.scale.z = 0.2
        # marker.color.r = 1.0
        # marker.color.g = 0.0
        # marker.color.b = 0.0
        # marker.color.a = 1.0
        # marker.lifetime = rospy.Duration()

        # marker_pub.publish(marker)
        # rate.sleep()

# if __name__ == '__main__':
    # try:
        # publish_marker()
    # except rospy.ROSInterruptException:
        # pass

# while not rospy.is_shutdown():
    # rospy.init_node('path_publisher')
    # path_publisher = rospy.Publisher("/path_topic", Path, queue_size=10)
    # rate = rospy.Rate(10)  # Publish at 10 Hz
    
    # path = Path()
    # path.header.stamp = rospy.Time.now()
    # path.header.frame_id = "map"

    # px = 0.0
    # py = 0.0
    # pz = 0.0
    # qx,qy,qz,qw = quaternion_from_euler(0,0,0)
    
    # for i in range(100):
        # px += 0.1
        # py += 0.1
        # pz += 0.0
        # pose = PoseStamped()
        # pose.header.stamp = rospy.Time.now()
        # pose.header.frame_id = "map"
        # pose.pose.position.x = px  # Increase the x-coordinate of the point
        # pose.pose.position.y = py  # Increase the y-coordinate of the point
        # pose.pose.position.z = pz  # Increase the z-coordinate of the point
        # pose.pose.orientation.x = qx
        # pose.pose.orientation.y = qy
        # pose.pose.orientation.z = qz
        # pose.pose.orientation.w = qw
        # path.poses.append(pose)
    # path_publisher.publish(path)
    # rate.sleep()














