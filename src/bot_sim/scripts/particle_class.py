#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler
import random
import numpy as np
import sensor_model as sensor
import motion_model as motion
import copy

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

    def setParticle(self,robotPose,i):
        self.particleArray[i] = robotPose

    def getParticle(self,i):
        return self.particleArray[i]
    
    def setParticleArray(self,newParticleArray):
        self.particleArray = newParticleArray

    def getParticleArray(self):
        return self.particleArray

    def ResampleParticles(self):
        # print("before resampling",self.particleArray)
        new_particles = []
        c = []
        c.append(self.weight[0])
        # print(self.weight)
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
            new_particles.append(copy.copy(self.getParticle(i)))
            u+=1/self.N
            j+=1
        for _ in range(self.N-len(new_particles)):
            new_particles.append(copy.copy(self.getParticle(self.N-1)))

        # for i in range(len(new_particles)):
            # x = copy.copy(new_particles[i][0])

        return new_particles

    def UpdateWeight(self):
            # print("BEFORE update weight")
            # print(self.particleArray)
            for i in range(self.N):
                self.weight[i] = self.sensor_model.CorrectionOneParticle(copy.copy(self.getParticle(i)))

            # print("AFTER update weight")
            # print(self.particleArray)

            sum_of_weight = sum(self.weight)
            # print("sum_of_weight",sum_of_weight)

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
pose = Pose()
clicked = False
def Clicked(msg):
    global pose
    global clicked

    pose = msg.pose.pose
    clicked = True
if __name__=='__main__':
    rospy.init_node("particle_class_p")

    delta_t = 0.05
    rate = rospy.Rate(1/delta_t)

    particleNum = 50
    particles = Particles(particleNum)
    particles.InitParticles()

    motion_model = motion.MotionModel(delta_t)
    sub = rospy.Subscriber("/initialpose",PoseWithCovarianceStamped,Clicked,queue_size=10)

    counter = 1
    while not rospy.is_shutdown():
        particles.PublishParticles()

        if counter == 100000:
            counter = 0
        if counter % 50 == 0:
            particles.UpdateWeight()
            # break
            particles.setParticleArray(particles.ResampleParticles())

        # Motion Update
        if motion_model.getMoving() == True:
            for i in range(particleNum):
                # print("particles.particleArray[0]",parrticles.getParticle(0))
                particles.setParticle(motion_model.PredictMotion(particles.getParticle(i)),i)
            counter += 1

        # print(particles.particleArray)
        rate.sleep()

# print(particles.weight)
# print(sum(particles.weight))
# print(particles.particleArray)
# particles.ResampleParticles()
# print(particles.particleArray)



