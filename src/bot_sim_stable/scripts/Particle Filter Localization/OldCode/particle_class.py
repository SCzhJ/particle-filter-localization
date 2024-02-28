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
import util

import copy

class Particles:
    def __init__(self,numOfParticles=50):
        self.util = util.Util()
        
        self.N = numOfParticles
        self.pub_poseArray = rospy.Publisher("particles",PoseArray,queue_size=10)
        self.poseArray = PoseArray()
        self.poseArray.header.frame_id = "odom"

        # [x,y,theta]
        self.particleArray = []

        # used to initalize particles
        self.gridMap = self.util.getMap()

        # particle weight
        self.weight = [0.0] * self.N

        self.sensor_model = sensor.SensorModel()

        self.init_lin_sigma = 0.3
        self.init_ang_sigma = 0.1
    
    def getParticleWeightArray(self):
        return self.weight

    def getParticleWeight(self,i):
        return self.weight[i]

    def setParticleWeight(self,i,w):
        self.weight[i] = w

    def setParticle(self,robotPose,i):
        self.particleArray[i] = robotPose

    def getParticle(self,i):
        return self.particleArray[i]
    
    def setParticleArray(self,newParticleArray):
        self.particleArray = newParticleArray

    def getParticleArray(self):
        return self.particleArray
    
    # input grid position, output resampled actual coordinate position [x,y,theta]
    def GridPosResActPos(self):
        valid = False
        while valid == False:
            x = random.randint(1,self.gridMap.info.width-1)
            y = random.randint(1,self.gridMap.info.height-1)
            if self.util.OccupancyCheckGridCoord(x,y)==False:
                valid = True
        x,y = self.util.GridPos2ActPos(x,y)
        angular_z = random.uniform(-np.pi,np.pi)
        return [x,y,angular_z]

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
            x,y = self.util.ActPos2GridPos(x,y)
            if self.util.OccupancyCheckGridCoord(x,y)==False:
                valid = True
            else:
                valid = False
                while valid == False:
                    x = random.randint(1,self.gridMap.info.width-1)
                    y = random.randint(1,self.gridMap.info.height-1)
                    if self.util.OccupancyCheckGridCoord(x,y)==False:
                        valid = True
                x,y = self.util.GridPos2ActPos(x,y)
                return [x,y,z]
        # print("particle ",i," initialized")
        x,y = self.util.GridPos2ActPos(x,y)
        return [x,y,z]
        
    def InitParticlesAroundPose(self,robotPose,Num):
        init_particles = []
        for i in range(Num):
            init_particles.append(self.SampleParticleAroundPose(robotPose,self.init_lin_sigma,self.init_ang_sigma))
        self.setParticleArray(init_particles)

    def InitParticles(self):
        init_particles = []
        for i in range(self.N):
            valid = False
            while valid == False:
                x = random.randint(1,self.gridMap.info.width-1)
                y = random.randint(1,self.gridMap.info.height-1)
                if self.util.OccupancyCheckGridCoord(x,y)==False:
                    valid = True
            # print("particle ",i," initialized")
            x,y = self.util.GridPos2ActPos(x,y)
            angular_z = random.uniform(0,2*np.pi)
            self.init_particles.append([x,y,angular_z])
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


    
# Test Program
if __name__=='__main__':
    rospy.init_node("particle_class_p")

    delta_t = 0.05
    rate = rospy.Rate(1/delta_t)

    Odom = est.GazeboOdom()

    particleNum = 80
    particles = Particles(particleNum)
    # particles.setParticleArray(particles.InitParticles())
    particles.setParticleArray(particles.InitParticlesAroundPose([Odom.pose2DEst.x,Odom.pose2DEst.y,Odom.pose2DEst.z],particleNum))
    particles.InitPose()

    motion_model = motion.MotionModel(delta_t)


    counter = 1
    w_avg = 0
    w_slow = 0
    w_fast = 0

    alpha_slow = 1
    alpha_fast = 50

    while not rospy.is_shutdown():
        particles.PublishParticles()

        if counter == 100000:
            counter = 0
        if counter % 30 == 0 and motion_model.getMoving():
            w_avg += particles.UpdateWeight()
            w_slow = w_slow + alpha_slow * (w_avg - w_slow)
            w_fast = w_fast + alpha_fast * (w_avg - w_fast)
            # break
            particles.setParticleArray(particles.ResampleParticlesWithVariation(max(0.0,1.0-w_fast/w_slow)))

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



