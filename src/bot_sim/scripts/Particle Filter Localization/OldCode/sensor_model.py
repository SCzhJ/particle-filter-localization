#!/usr/bin/env python3

import rospy
import raycast as rc
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from tf.transformations import euler_from_quaternion
import numpy as np
import util
import scipy
from scipy.ndimage import gaussian_filter
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from std_msgs.msg import Float64
import drifter_class as dft
import copy

class SensorModel:
    def __init__(self):
        self.util = util.Util()
        # given location, probability of gaining the current measurement
        prob = 0
        self.rc = rc.Raycast()

        self.mapMinusOneSub = 13
        self.gridMap = self.util.getMap()
        self.gridMap2D = self.FoldMap()

        self.gaussFilterSigma = 10.0
        self.LikelihoodField = self.GenerateLikelihoodField()
        self.OverlapMap = self.OverlapFieldAndGrid()

        self.interval = 30
        # standard deviation of our sensor model
        self.sigma_r = 4.0 
        self.N = 1

    def FieldLikelihood(self,x_end,y_end):
        x_end,y_end = self.util.ActPos2GridPos(x_end,y_end)
        if x_end >= self.gridMap.info.width or y_end >= self.gridMap.info.height:
            return 0
        else:
            return self.LikelihoodField[y_end][x_end]/100


    # robotPose: [x,y,theta]
    def CorrectionOneParticleFieldMethod(self,robotPose):
        total_likelihood = np.longdouble(1.0)
        for i in range(self.rc.scan_num):
            if i % self.interval == 0:
                range_i = self.rc.getRanges()[i]
                x_end = range_i * np.cos(robotPose[2]+self.rc.scan.angle_increment*i+self.rc.scan.angle_min)+robotPose[0]
                y_end = range_i * np.sin(robotPose[2]+self.rc.scan.angle_increment*i+self.rc.scan.angle_min)+robotPose[1]
                measurement_likelihood = np.longdouble(self.FieldLikelihood(x_end,y_end))
                print("robot angle",robotPose[2])
                print("sense angle",robotPose[2]+self.rc.scan.angle_increment*i+self.rc.scan.angle_min)
                print("robot x y",robotPose[0],robotPose[1])
                print("point x y",x_end,y_end)
                print(measurement_likelihood)
                total_likelihood *= measurement_likelihood
        return total_likelihood
        

    def GenerateLikelihoodField(self):
        new_map = gaussian_filter(self.gridMap2D,sigma=self.gaussFilterSigma)
        max_element = 0
        for i in range(self.gridMap.info.height):
            max_element = max(max(new_map[i]),max_element)
        for i in range(self.gridMap.info.height):
            for j in range(self.gridMap.info.width):
                if self.gridMap.data[i*self.gridMap.info.width+j] != 0:
                    new_map[i][j] = int(100*new_map[i][j]/max_element)
                else:
                    new_map[i][j] = 1
        return new_map
    
    def OverlapFieldAndGrid(self):
        new_map = copy.deepcopy(self.LikelihoodField)
        for i in range(self.gridMap.info.height):
            for j in range(self.gridMap.info.width):
                if self.gridMap.data[i*self.gridMap.info.width+j] != 0:
                    new_map[i][j] = self.gridMap.data[i*self.gridMap.info.width+j]
        return new_map


    def DefoldMap(self,map2D):
        map1D = []
        for i in range(self.gridMap.info.height):
            map1D = map1D + list(map2D[i])
        return map1D

    def FoldMap(self):
        map2D = []
        for i in range(self.gridMap.info.height):
            map2D.append(list(self.gridMap.data[i*self.gridMap.info.width:(i+1)*self.gridMap.info.width]))
        for i in range(self.gridMap.info.height):
            for j in range(self.gridMap.info.width):
                if map2D[i][j] == -1:
                    map2D[i][j] = self.mapMinusOneSub
        return map2D

    # robotPose: [x,y,theta]
    def CorrectionOneParticle(self,robotPose):
        rays = self.rc.RaycastDDA(robotPose,self.interval)
        total_likelihood = 1.0
        for i in range(len(rays)):
            measurement_likelihood = self.NormalPDFtimesN(rays[i],self.rc.getRanges()[i*self.interval],self.N)
            total_likelihood *= measurement_likelihood
        return total_likelihood
    
    def NormalPDFtimesN(self,mu,x,N):
        denom = self.sigma_r*np.sqrt(2*np.pi)
        num = self.N*np.exp(-(x-mu)**2/(2 * (self.sigma_r**2)))
        return num/denom


# Test Program


# if __name__=='__main__':
    # rospy.init_node("sensor_model_p")
    # sensor_model = SensorModel()
    # pub = rospy.Publisher("/map_new",OccupancyGrid,queue_size=10)
    # rate = rospy.Rate(10)

    # map1D = sensor_model.DefoldMap(sensor_model.LikelihoodField)
    # mapPub = OccupancyGrid()
    # mapPub.header.frame_id = "map"
    # mapPub.header.stamp = rospy.Time.now()
    # mapPub.info = sensor_model.gridMap.info
    # mapPub.data = map1D

    # while not rospy.is_shutdown():
        # pub.publish(mapPub)
        # rate.sleep()

# Previous Test Program

pose = Pose()
clicked = False
def Clicked(msg):
    global pose
    global clicked
    pose = msg.pose.pose
    clicked = True

if __name__=="__main__":
    rospy.init_node("sensor_model_p")
    sensor_model = SensorModel()
    sub = rospy.Subscriber("/initialpose",PoseWithCovarianceStamped,Clicked,queue_size=10)

    pub = rospy.Publisher("/point_likelihood",Float64,queue_size=10)
    rate = rospy.Rate(20)
    likelihood = -1
    while not rospy.is_shutdown():
        if clicked == True:
            clicked = False
            orientation_list = [
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w]
            angular_z = euler_from_quaternion(orientation_list)[2]
            likelihood = sensor_model.CorrectionOneParticleFieldMethod([pose.position.x,pose.position.y,angular_z])
            pub.publish(likelihood)
        print("point likelihood:",likelihood)
        rate.sleep()

