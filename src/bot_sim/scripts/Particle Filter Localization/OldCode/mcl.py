#!/usr/bin/env python3
import rospy
import particle_class as par
import sensor_model as sensor
import motion_model as motion
import util
import copy
import est_eval_class as est
import numpy as np
import random

class MCL:
    def __init__(self,numOfParticles):
        self.util = util.Util()
        self.ParClass = par.Particles(numOfParticles)
        self.N = numOfParticles

        self.res_lin_sigma = 0.2
        self.res_ang_sigma = 0.1

        self.mov_lin_sigma = 0.1
        self.mov_ang_sigma = 0.1
        self.mov_err_list = []

        self.resampleLimit = int(0.3 * numOfParticles)

    def getAngVelErr(self,i):
        return self.mov_err_list[i][1]

    def getLinVelErr(self,i):
        return self.mov_err_list[i][0]

    def ResampleMoveErr(self):
        self.mov_err_list = []
        for i in range(self.N):
            self.mov_err_list.append([
                np.random.normal(0.0,self.mov_lin_sigma),
                np.random.normal(0.0,self.mov_ang_sigma)])

    # Augmented Particle filter, 
    # resample with probability "prob" of Random Sampling
    # if normal Particle filter, set prob to 0
    def ResampleParticlesWithVariation(self,prob):
        new_particles = []
        c = []
        c.append(self.ParClass.getParticleWeight(0))
        for i in range(1,self.N):
            c.append(c[i-1]+self.ParClass.getParticleWeight(i))

        u = np.random.uniform(0,1/self.N)
        i = 0
        j = 0
        prev_i = 0
        count = 0 
        end = False
        while j<self.N:
            while u>c[i]:
                i+=1
                if i>=self.N:
                    i-=1
                    j=self.N
                    break
            if random.uniform(0,1) < prob:
                new_particles.append(self.ParClass.GridPosResActPos())
            else:
                if i==prev_i:
                    count += 1
                    if count > self.resampleLimit:
                        new_particles.append(self.ParClass.GridPosResActPos())
                else:
                    prev_i = i
                    new_particles.append(self.ParClass.SampleParticleAroundPose(copy.copy(self.ParClass.getParticle(i)),self.res_lin_sigma,self.res_ang_sigma))
            u+=1/self.N
            j+=1
        for _ in range(self.N-len(new_particles)):
            new_particles.append(self.ParClass.SampleParticleAroundPose(copy.copy(self.ParClass.getParticle(self.N-1)),self.res_lin_sigma,self.res_ang_sigma))

        for i in range(self.N):
            x = copy.copy(new_particles[i][0])
            y = copy.copy(new_particles[i][1])
            x,y = self.util.ActPos2GridPos(x,y)
            if self.util.OccupancyCheckGridCoord(x,y)==True:
                new_particles[i] = self.ParClass.GridPosResActPos()
                print(new_particles[i])

        self.ParClass.setParticleArray(new_particles)

    def UpdateWeight(self):
        for i in range(self.N):
            self.ParClass.setParticleWeight(i,self.ParClass.sensor_model.CorrectionOneParticleFieldMethod(copy.copy(self.ParClass.getParticle(i))))
        sum_of_weight = sum(self.ParClass.getParticleWeightArray())
        for i in range(self.N):
            self.ParClass.setParticleWeight(i,
                    self.ParClass.getParticleWeight(i)/sum_of_weight)
        return sum_of_weight/self.N

# Test Program
if __name__=='__main__':
    rospy.init_node("mcl_p")

    delta_t = 0.05
    rate = rospy.Rate(1/delta_t)

    Odom = est.GazeboOdom()

    particleNum = 180
    mcl = MCL(particleNum)

    mcl.ParClass.InitParticlesAroundPose([Odom.pose2DEst.x,Odom.pose2DEst.y,Odom.pose2DEst.z],particleNum)
    mcl.ResampleMoveErr()
    mcl.ParClass.InitPose()
    motion_model = motion.MotionModel(delta_t)

    counter = 1
    w_avg = 0
    # w_slow = 0
    # w_fast = 0

    # alpha_slow = 1
    # alpha_fast = 5

    while not rospy.is_shutdown():
        mcl.ParClass.PublishParticles()

        if counter == 100000:
            counter = 0
        if counter % 30 == 0 and motion_model.getMoving():
            w_avg += mcl.UpdateWeight()
            # w_slow = w_slow + alpha_slow * (w_avg - w_slow)
            # w_fast = w_fast + alpha_fast * (w_avg - w_fast)
            # mcl.ResampleParticlesWithVariation(max(0.0,1.0-w_fast/w_slow))
            mcl.ResampleParticlesWithVariation(0)
            mcl.ResampleMoveErr()

        # Motion Update
        if motion_model.getMoving() == True:
            for i in range(particleNum):
                mcl.ParClass.setParticle(motion_model.PredictMotion(
                    mcl.ParClass.getParticle(i),
                    mcl.getLinVelErr(i),
                    mcl.getAngVelErr(i)),i)
            counter += 1

        rate.sleep()
