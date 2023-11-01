#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
import particle_class as pars
import motion_model as motion
import numpy as np

pose = Pose()
clicked = False
def Clicked(msg):
    global pose
    global clicked

    pose = msg.pose.pose
    clicked = True

if __name__=='__main__':
    rospy.init_node("mcl_p")

    # time interval, delta time
    delta_t = 0.05

    rate = rospy.Rate(1/delta_t)
    particleNum = 50
    particles = pars.Particles(particleNum)
    particles.InitParticles()

    motion_model = motion.MotionModel(delta_t)
    sub = rospy.Subscriber("/initialpose",PoseWithCovarianceStamped,Clicked,queue_size=10)

    counter = 0

    while not rospy.is_shutdown():

        particles.PublishParticles()

        if counter == 100000:
            counter = 0
        if clicked == True:
            clicked = False
            particles.UpdateWeight()
            particles.ResampleParticles()

        # Motion Update
        for i in range(len(particles.particleArray)):
            particles.particleArray[i] = motion_model.PredictMotion(particles.particleArray[i])

        counter+=1
        rate.sleep()

