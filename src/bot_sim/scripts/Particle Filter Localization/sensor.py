#!/usr/bin/env python3

import rospy
import pickle
from sensor_msgs.msg import LaserScan
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float64
import util
from decimal import *
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32

class SensorModel:
    def __init__(self,likelihood_field_file_name="LikelihoodField/likelihood_field_sigma_2"):
        self.util = util.Util()
        self.grid_info = self.util.getMapInfo()
        self.field_file_name = likelihood_field_file_name

        # the likelihood of empty space (-1 in grid map),
        # it is set by ReadLikelihoodField()
        self.empty_likelihood = 0.00001

        # the likelihood field in 1D 
        # (in the same dimension of OccupancyGrid's data entry)
        self.field = []
        self.ReadLikelihoodField()

        # the topic of our scan, should change it to rosparam later
        self.scan_topic = "/scan"
        self.sub_laser = rospy.Subscriber(self.scan_topic,LaserScan,self.RcvScan,queue_size=10)
        self.scan = LaserScan()

        # apply 1/m smoothing over all the weights
        # i.e. prob**(1/m)
        self.m = 1000

    # robot_pose is dict() with x, y, theta
    def CalcPoseSmoothedLikelihood(self,robot_pose):
        return np.exp(1/self.m * self.CalcPoseLogLikelihood(robot_pose))

    # robot_pose is dict() with x, y, theta
    def CalcPoseLogLikelihood(self,robot_pose):
        angle = self.util.WrapToPosNegPi(self.scan.angle_min)
        total_log_likelihood = 0.0
        # for testing
        # point_cloud = []
        for i in range(len(self.scan.ranges)):

            # for given robot pose, calculate where the expected laser will hit
            x = robot_pose["x"] + self.scan.ranges[i] * np.cos(self.util.WrapToPosNegPi(robot_pose["theta"] + angle))
            y = robot_pose["y"] + self.scan.ranges[i] * np.sin(self.util.WrapToPosNegPi(robot_pose["theta"] + angle))

            # the following is due to properties of LaserScan message
            angle += self.scan.angle_increment

            # the following lines are for testing
            # print("angles",i,": ",angle,"x-y: ",x,y)
            # the following will be later returned in test program
            # point_cloud.append([x,y])

            x,y = self.util.ActPos2GridPos(x,y)

            # if the expected laser ends at empty space, 
            # use the minimum likelihood in the map
            if x<0 or y<0 or x>=self.util.grid_info.width or y>=self.util.grid_info.height:
                log_likelihood = np.log(self.empty_likelihood)
            else:
                try:
                    log_likelihood = np.log(self.field[y*self.grid_info.width+x])
                except Exception as e:
                    print(e)
                    print(self.field[y*self.grid_info.width+x])
                    print(print(self.util.GridPos2ActPos(x,y)))

            # multiply all the probabilities together,
            # which is equivalent to adding the log likelihoods
            total_log_likelihood += log_likelihood

        return total_log_likelihood

    def ReadLikelihoodField(self):
        with open(self.field_file_name,'rb') as f:
            self.field = pickle.load(f)

        min_except_zero = 1.0
        for i in range(len(self.field)):
            if self.field[i]<min_except_zero and self.field[i]!=0:
                min_except_zero = self.field[i]
        self.empty_likelihood = min_except_zero
        self.field[self.field==0]=min_except_zero
        for i in range(len(self.field)):
            if self.field[i]==0:
                rospy.logerr("field entry still zero!")

    def RcvScan(self,msg):
        self.scan = msg


# Test Program

pose = Pose()
clicked = False
def Clicked(msg):
    global pose
    global clicked
    pose = msg.pose.pose
    clicked = True

if __name__=="__main__":
    rospy.init_node("sensor_model_p")
    rate = rospy.Rate(5)

    sensor = SensorModel("LikelihoodField/likelihood_field_sigma_2")

    sub = rospy.Subscriber("/initialpose",PoseWithCovarianceStamped,Clicked,queue_size=10)

    pub = rospy.Publisher("/point_likelihood",Float64,queue_size=10)

    likelihood = -1

    robot_pose = dict()
    robot_pose["x"] = 0
    robot_pose["y"] = 0
    robot_pose["theta"] = 0

    pub_point_cloud = rospy.Publisher("/point_cloud_c",PointCloud,queue_size=10)
    point_cloud = PointCloud()
    point_cloud.header.frame_id = "map"

    print("min field:",sensor.empty_likelihood)

    while not rospy.is_shutdown():
        if clicked == True:
            clicked = False
            orientation_list = [
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w]
            robot_pose["theta"] = euler_from_quaternion(orientation_list)[2]
            robot_pose["x"] = pose.position.x
            robot_pose["y"] = pose.position.y
            print("x: ",robot_pose["x"],"y: ",robot_pose["y"])
            print("angle:", robot_pose["theta"])
            likelihood = sensor.CalcPoseSmoothedLikelihood(robot_pose)
            pub.publish(likelihood)

            # point_cloud.header.stamp = rospy.Time.now()
            # list_of_points = []
            # for i in range(len(points)):
                # point = Point32()
                # point.x = points[i][0]
                # point.y = points[i][1]
                # point.z = 0.5
                # list_of_points.append(point)
            # point_cloud.points = list_of_points
            # pub_point_cloud.publish(point_cloud)
            
        print("log likelihood:",likelihood,"1/m likelihood:",likelihood)
        rate.sleep()

















