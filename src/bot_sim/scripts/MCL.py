#!/usr/bin/env python3

import rospy
import motion
import sensor
import particles
import util

import numpy as np

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

if __name__=="__main__":
    rospy.init_node("mcl_p")
    dt = 0.05
    rate = rospy.Rate(1/dt)

    util = util.Util()

    par_num = 200
    particles = particles.Particles(par_num)
    sensor = sensor.SensorModel("LikelihoodField/likelihood_field_sigma_2")
    motion = motion.MotionModel(dt)

    # get the current position
    current_pose = rospy.wait_for_message("/odom",Odometry)
    c_pose = current_pose.pose.pose

    # turn Odom into dict of x, y, theta
    init_pose = dict()
    init_pose["x"] = c_pose.position.x
    init_pose["y"] = c_pose.position.y
    orient_list = [c_pose.orientation.x,c_pose.orientation.y,c_pose.orientation.z,c_pose.orientation.w]
    init_pose["theta"] = euler_from_quaternion(orient_list)[2]

    # initialize particles around current pose
    particles.ResampleAllParticlesAroundPose(init_pose)

    counter = 1

    while not rospy.is_shutdown():
        # publish particle pose
        particles.PublishParticles()

        if counter >= 100000:
            counter = 0

        if counter % 20 == 0 and motion.getMoving():
            ##########################
            # Particle Update Weight #
            ##########################
            '''
            :put new weight into particle class
            '''
            sum_likelihood = 0
            for i in range(par_num):
                likelihood = sensor.CalcPoseSmoothedLikelihood(particles.getParticle(i))
                sum_likelihood += likelihood
                particles.setWeight(i,likelihood)

            #######################
            # Particle Resampling #
            #######################
            '''
            Resample all the particles according to the weight
            When a particle is selected, resample it with noise
            '''
            step = 1/par_num
            u = np.random.uniform(0,step)
            c = particles.getWeight(0)/sum_likelihood

            i = 0
            new_particles = []
            for j in range(par_num):
                while u>c:
                    i+=1
                    c+=particles.getWeight(i)/sum_likelihood
                new_particles.append(particles.getParticle(i))
                u+=step
            # diff = par_num - len(new_particles)
            # if diff > 0:
                # for i in range(diff):
                    # rand_i = np.random.randint(0,par_num-diff)
                    # new_particles.append(new_particles[rand_i])
            # elif diff < 0:
                # rospy.logerr("new_particles more than particle number!")

            particles.setParticles(new_particles)

            #############################
            # Velocity Error Resampling #
            #############################
            '''
            Resample all the motion noise 
            -- motion noise for one particle maintains the same before next resampling
            '''
            # particles.ResampleVelocityNoise()

        # Predict Motion
        '''put new position into particle class'''
        motion.UpdateVelocity()
        for i in range(par_num):
            new_par_pos = motion.PredictMotionJoint(
                particles.getParticle(i),0.3,0.3)
            particles.setParticle(i,new_par_pos)

        counter += 1
        rate.sleep()
