#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
import random
import numpy as np
import sensor_model as sensor

class Particles:
    def __init__(self,numOfParticles=50):
        
        self.N = numOfParticles

        self.pub_poseArray = rospy.Publisher("particles",PoseArray,queue_size=10)
        self.poseArray = PoseArray()
        self.poseArray.header.frame_id = "odom"

        # [x,y,theta]
        self.particleArray = []

        # used to initalize particles
        self.gridMap = self.LoadMap()

        # particle weights
        self.weight = [0.0] * self.N

        self.sensor_model = sensor.SensorModel()

    def ResampleParticles(self):
        new_particles = []
        c = []
        c.append(self.weight[0])
        print(self.weight)
        for i in range(1,self.N):
            c.append(c[i-1]+self.weight[0])

        u = np.random.uniform(0,1/self.N)
        i = 0
        j = 0
        end = False
        while j<self.N:
            while u>c[i]:
                i+=1
                if i>=self.N:
                    i-=1
                    j=self.N
                    break
            new_particles.append(self.particleArray[i])
            u+=1/self.N
            j+=1
        for _ in range(self.N-len(new_particles)):
            new_particles.append(self.particleArray[self.N-1])
        self.particleArray = new_particles

    def UpdateWeight(self):
            for i in range(self.N):
                self.weight[i] = self.sensor_model.CorrectionOneParticle(self.particleArray[i])

            sum_of_weight = sum(self.weight)

            for i in range(self.N):
                self.weight[i] = self.weight[i]/sum_of_weight

    def PublishParticles(self):
        for i in range(self.N):
            self.poseArray.poses[i].position.x = self.particleArray[i][0]
            self.poseArray.poses[i].position.y = self.particleArray[i][1]
            [qx,qy,qz,qw] = quaternion_from_euler(0,0,self.particleArray[i][2])
            self.poseArray.poses[i].orientation.z = qz
            self.poseArray.poses[i].orientation.w = qw
        self.pub_poseArray.publish(self.poseArray)

    def InitParticles(self):
        for i in range(self.N):
            valid = False
            while valid == False:
                x = random.randint(1,self.gridMap.info.width-1)
                y = random.randint(1,self.gridMap.info.height-1)
                if self.OccupancyCheck(x,y)==False:
                    valid = True
            # print("particle ",i," initialized")
            x *= self.gridMap.info.resolution
            y *= self.gridMap.info.resolution
            x += self.gridMap.info.origin.position.x
            y += self.gridMap.info.origin.position.y
            angular_z = random.uniform(0,2*np.pi)
            self.particleArray.append([x,y,angular_z])

        poses = []
        pose = Pose()
        for i in range(len(self.particleArray)):
            pose.position.x = self.particleArray[i][0]
            pose.position.y = self.particleArray[i][1]
            pose.position.z = 0.3
            [qx,qy,qz,qw] = quaternion_from_euler(0,0,self.particleArray[i][2])
            pose.orientation.x = qx
            pose.orientation.y = qy
            pose.orientation.z = qz
            pose.orientation.w = qw
            poses.append(pose)
            pose = Pose()
        self.poseArray.poses = poses


    def OccupancyCheck(self,x,y):

        if x > self.gridMap.info.width or y > self.gridMap.info.height or x < 0 or y < 0:
            return True
        occ = self.gridMap.data[y*self.gridMap.info.width+x]
        if occ == -1:
            return True
        else:
            return False
    

    def LoadMap(self):
        rospy.wait_for_service('/static_map') 
        gridMapService = rospy.ServiceProxy('/static_map',GetMap)
        response = gridMapService()
        # each grid of the map is from 0 to 100. For our algorithm, 
        # if the grid has value greater then 50 we consider it as occupied

        # this number is grid [0,0] location relative to Map's Transform Frame (0,0) location
        # in other words, location of grid[0,0] under map Frame's transform coordinate
        # this origin information includes [x,y,theta] in unit of [m,m,rad]
        self.originRelativePose = response.map.info.origin
        print("map loaded")
        return response.map

# Test Program
if __name__=='__main__':
    rospy.init_node("particle_class_p")
    rate = rospy.Rate(1)
    Particles = Particles(50)
    Particles.InitParticles()

    while not rospy.is_shutdown():
        Particles.PublishParticles()
        rate.sleep()


