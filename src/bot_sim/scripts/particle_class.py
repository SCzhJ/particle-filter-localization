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
import est_eval_class as est
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

        self.init_lin_sigma = 0.3
        self.init_ang_sigma = 0.1

        self.res_lin_sigma = 0.3
        self.res_ang_sigma = 0.3

        self.mov_lin_sigma = 0.15
        self.mov_ang_sigma = 0.2
        self.mov_err_list = []

    def getParticleAngErr(self,i):
        return self.mov_err_list[i][1]

    def getParticleLinErr(self,i):
        return self.mov_err_list[i][0]

    def setParticle(self,robotPose,i):
        self.particleArray[i] = robotPose

    def getParticle(self,i):
        return self.particleArray[i]
    
    def setParticleArray(self,newParticleArray):
        self.particleArray = newParticleArray

    def getParticleArray(self):
        return self.particleArray

    def ResampleParticlesWithVariation(self):
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
            new_particles.append(self.SampleParticleAroundPose(copy.copy(self.getParticle(i)),self.res_lin_sigma,self.res_ang_sigma))
            u+=1/self.N
            j+=1
        for _ in range(self.N-len(new_particles)):
            new_particles.append(self.SampleParticleAroundPose(copy.copy(self.getParticle(self.N-1)),self.res_lin_sigma,self.res_ang_sigma))

        for i in range(self.N):
            x = copy.copy(new_particles[i][0])
            y = copy.copy(new_particles[i][1])
            x -= self.gridMap.info.origin.position.x
            y -= self.gridMap.info.origin.position.y
            x = int(x/self.gridMap.info.resolution)
            y = int(y/self.gridMap.info.resolution)
            if self.OccupancyCheck(x,y)==True:
                valid = False
                while valid == False:
                    x = random.randint(1,self.gridMap.info.width-1)
                    y = random.randint(1,self.gridMap.info.height-1)
                    if self.OccupancyCheck(x,y)==False:
                        valid = True
                x *= self.gridMap.info.resolution
                y *= self.gridMap.info.resolution
                x += self.gridMap.info.origin.position.x
                y += self.gridMap.info.origin.position.y
                angular_z = random.uniform(0,2*np.pi)
                new_particles[i] = [x,y,angular_z]

        self.mov_err_list = []
        for i in range(self.N):
            self.mov_err_list.append([
                np.random.normal(0.0,self.mov_lin_sigma),
                np.random.normal(0.0,self.mov_ang_sigma)])

        return new_particles

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

        return new_particles

    def UpdateWeight(self):
            for i in range(self.N):
                self.weight[i] = np.sqrt(self.sensor_model.CorrectionOneParticle(copy.copy(self.getParticle(i))))

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

    def SampleParticleAroundPose(self,robotPose,lin_sigma,ang_sigma):
        valid = False
        z = np.random.normal(robotPose[2],ang_sigma)
        while valid == False:
            x = np.random.normal(robotPose[0],lin_sigma)
            y = np.random.normal(robotPose[1],lin_sigma)
            x -= self.gridMap.info.origin.position.x
            y -= self.gridMap.info.origin.position.y
            x = int(x/self.gridMap.info.resolution)
            y = int(y/self.gridMap.info.resolution)
            if self.OccupancyCheck(x,y)==False:
                valid = True
            else:
                print("occ True")
                valid = False
                while valid == False:
                    x = random.randint(1,self.gridMap.info.width-1)
                    y = random.randint(1,self.gridMap.info.height-1)
                    if self.OccupancyCheck(x,y)==False:
                        valid = True
                x *= self.gridMap.info.resolution
                y *= self.gridMap.info.resolution
                x += self.gridMap.info.origin.position.x
                y += self.gridMap.info.origin.position.y
                return [x,y,z]
        # print("particle ",i," initialized")
        x *= self.gridMap.info.resolution
        y *= self.gridMap.info.resolution
        x += self.gridMap.info.origin.position.x
        y += self.gridMap.info.origin.position.y
        return [x,y,z]
        
    def InitParticlesAroundPose(self,robotPose,Num):
        init_particles = []
        for i in range(Num):
            init_particles.append(self.SampleParticleAroundPose(robotPose,self.init_lin_sigma,self.init_ang_sigma))

        self.mov_err_list = []
        for i in range(self.N):
            self.mov_err_list.append([
                np.random.normal(0.0,self.mov_lin_sigma),
                np.random.normal(0.0,self.mov_ang_sigma)])

        return init_particles

    def InitParticles(self):
        init_particles = []
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
            self.init_particles.append([x,y,angular_z])

        self.mov_err_list = []
        for i in range(self.N):
            self.mov_err_list.append([
                np.random.normal(0.0,self.mov_lin_sigma),
                np.random.normal(0.0,self.mov_ang_sigma)])


        return init_particles

    def InitPose(self):
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
        if occ == -1 or occ>90:
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

    delta_t = 0.05
    rate = rospy.Rate(1/delta_t)

    Odom = est.GazeboOdom()

    particleNum = 100
    particles = Particles(particleNum)
    # particles.setParticleArray(particles.InitParticles())
    particles.setParticleArray(particles.InitParticlesAroundPose([Odom.pose2DEst.x,Odom.pose2DEst.y,Odom.pose2DEst.z],particleNum))
    particles.InitPose()

    motion_model = motion.MotionModel(delta_t)


    counter = 1
    while not rospy.is_shutdown():
        particles.PublishParticles()

        if counter == 100000:
            counter = 0
        if counter % 30 == 0 and motion_model.getMoving():
            particles.UpdateWeight()
            # break
            particles.setParticleArray(particles.ResampleParticlesWithVariation())

        # Motion Update
        if motion_model.getMoving() == True:
            for i in range(particleNum):
                # print("particles.particleArray[0]",parrticles.getParticle(0))
                particles.setParticle(motion_model.PredictMotion(particles.getParticle(i),particles.getParticleLinErr(i),particles.getParticleAngErr(i)),i)
            counter += 1

        # print(particles.particleArray)
        rate.sleep()

# print(particles.weight)
# print(sum(particles.weight))
# print(particles.particleArray)
# particles.ResampleParticles()
# print(particles.particleArray)



