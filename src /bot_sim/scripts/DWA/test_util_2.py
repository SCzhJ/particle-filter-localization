#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped, PointStamped, Twist, PoseArray, Pose, Point, PoseWithCovarianceStamped, Transform
from nav_msgs.msg import Path
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from typing import List
import tf2_ros

class OdomSubscriber:
    def __init__(self):
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.robot_pose = None

    def odom_callback(self, msg):
        self.robot_pose = msg.pose.pose

    def get_pose(self):
        if self.robot_pose is None:
            return None, None, None
        x = self.robot_pose.position.x
        y = self.robot_pose.position.y
        theta = euler_from_quaternion([self.robot_pose.orientation.x, 
                                       self.robot_pose.orientation.y, 
                                       self.robot_pose.orientation.z, 
                                       self.robot_pose.orientation.w])[2]
        return x, y, theta

class Base_footprint_pos:
    def get_pose(self):
        # print("sdfadfasfsdfsd")
        try:
            trans = self.tfBuffer.lookup_transform(self.target_frame, self.source_frame, rospy.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            theta = euler_from_quaternion([trans.transform.rotation.x,trans.transform.rotation.y,trans.transform.rotation.z,trans.transform.rotation.w])[2]
            self.robot_pose=Point(x , y, theta)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            # rate.sleep()
            pass
    # def timer(self):
    #     rospy.Timer(rospy.Duration(0.05), self.listen_transform)
    #     rospy.spin()
    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.target_frame = rospy.get_param('~fixed_frame')
        self.source_frame = rospy.get_param('~base_foot_print')
        self.robot_pose=Point()
        # print("??????????????????????????????????????????????")
        # self.timer()
        # print("_________________________________________________")
    # def get_pose(self):
    #     trans = self.tfBuffer.lookup_transform(fixed_frame, base_foot_print, rospy.Time())
    #     x = trans.transform.translation.x
    #     y = trans.transform.translation.y
    #     theta = euler_from_quaternion([trans.transform.rotation.x,trans.transform.rotation.y,trans.transform.rotation.z,trans.transform.rotation.w])[2]
    #     return Point(x, y, theta)
class PointListPublisher:
    def __init__(self, marker_id: int = 5, frame_id: str = "base_footprint", topic_name: str = 'point_list_marker',
                  r: float = 1.0, g: float = 0.0, b: float = 0.0, a: float = 1.0,
                  x_scale: float = 0.02, y_scale: float = 0.02, z_scale: float = 0.02):
        self.marker_pub = rospy.Publisher(topic_name, Marker, queue_size=10)
        self.marker = Marker()
        self.marker.header.frame_id = frame_id
        self.marker.header.stamp = rospy.Time.now()
        self.marker.ns = "points"
        self.marker.id = marker_id
        self.marker.type = Marker.POINTS
        self.marker.action = Marker.ADD
        self.marker.scale.x = x_scale
        self.marker.scale.y = y_scale
        self.marker.scale.z = z_scale
        self.marker.color.r = r
        self.marker.color.g = g
        self.marker.color.b = b
        self.marker.color.a = a
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0
        self.marker.lifetime = rospy.Duration()

    def publish_point_list(self, point_list):
        self.marker.points = point_list
        self.marker_pub.publish(self.marker)

class PathPublisher:
    def __init__(self):
        self.path_pub = rospy.Publisher('path_topic', Path, queue_size=10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"
    
    def calc_path_from_point_list(self, point_list):
        self.path_msg.poses = []
        for i in range(len(point_list)):
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "map"
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.pose.position.x = point_list[i].x
            pose_stamped.pose.position.y = point_list[i].y
            q = quaternion_from_euler(0, 0, point_list[i].z)
            pose_stamped.pose.orientation.x = q[0]
            pose_stamped.pose.orientation.y = q[1]
            pose_stamped.pose.orientation.z = q[2]
            pose_stamped.pose.orientation.w = q[3]
            self.path_msg.poses.append(pose_stamped)
        self.path_msg.poses.append(pose_stamped)

    def publish_path(self):
        self.path_msg.header.stamp = rospy.Time.now()
        self.path_pub.publish(self.path_msg)

class ClickedPointSubscriber:
    def __init__(self):
        self.clicked_point_sub = rospy.Subscriber('/clicked_point', PointStamped, self.clicked_point_callback)
        self.clicked_point = None
        self.clicked = False

    def clicked_point_callback(self, msg):
        self.clicked_point = msg.point
        self.clicked = True
        rospy.loginfo("Clicked point received: %s", self.clicked_point)
    
    def set_clicked_false(self):
        self.clicked = False
    
    def get_clicked(self):
        return self.clicked
    
    def get_clicked_point(self):
        return self.clicked_point

class PointStampedPublisher:
    def __init__(self, topic_name: str = 'point_topic'):
        self.point_pub = rospy.Publisher(topic_name, PointStamped, queue_size=10)

    def publish_point(self, point):
        point_stamped = PointStamped()
        point_stamped.header.stamp = rospy.Time.now()
        point_stamped.header.frame_id = "map"
        point_stamped.point = point
        self.point_pub.publish(point_stamped)

class CostVisualizer:
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.cost_list = []

    def update(self, frame):
        self.ax.clear()
        self.ax.plot(self.cost_list)
        self.ax.set_title('Cost over time')
        self.ax.set_xlabel('Time')
        self.ax.set_ylabel('Cost')

    def visualize(self, cost_list):
        self.cost_list = cost_list
        ani = FuncAnimation(self.fig, self.update, frames=range(len(self.cost_list)))
        plt.show(block=False)
        plt.pause(0.1)

class CmdVelPublisher:
    def __init__(self, topic_name: str = 'cmd_vel'):
        self.cmd_vel_pub = rospy.Publisher(topic_name, Twist, queue_size=10)

    def publish_cmd_vel(self, linear_velocity, angular_velocity):
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_velocity
        cmd_vel.angular.z = angular_velocity
        self.cmd_vel_pub.publish(cmd_vel)

class PoseArrayPublisher:
    def __init__(self, topic_name: str = 'pose_array'):
        self.pose_array_pub = rospy.Publisher(topic_name, PoseArray, queue_size=10)
        self.pose_list = []

    def publish_pose_array(self):
        pose_array = PoseArray()
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = "map"
        pose_array.poses = self.pose_list
        self.pose_array_pub.publish(pose_array)    
    
    def pose_list_reset(self):
        self.pose_list = []
    
    # input np.ndarray[3,1], x, y, theta, representing robot pose
    def calc_pose_add_list(self, robot_pose):
        pose = Pose()
        pose.position.x = robot_pose[0][0]
        pose.position.y = robot_pose[1][0]
        q = quaternion_from_euler(0, 0, robot_pose[2][0])
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        self.pose_list.append(pose)

    def calc_point_add_list(self, point):
        pose = Pose()
        pose.position.x = point.x
        pose.position.y = point.y
        q = quaternion_from_euler(0, 0, point.z)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        self.pose_list.append(pose)

    
class InitialposeSubscriber:
    def __init__(self):
        self.initialpose_sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.initialpose_callback)
        self.initialpose = None

    def initialpose_callback(self, msg):
        self.initialpose = msg.pose.pose

    def get_pose_transform(self):
        tras = Transform()
        tras.translation.x = self.initialpose.position.x
        tras.translation.y = self.initialpose.position.y
        tras.translation.z = self.initialpose.position.z
        tras.rotation.x = self.initialpose.orientation.x
        tras.rotation.y = self.initialpose.orientation.y
        tras.rotation.z = self.initialpose.orientation.z
        tras.rotation.w = self.initialpose.orientation.w
        return tras