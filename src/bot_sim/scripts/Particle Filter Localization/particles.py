#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseArray, Pose
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import util
import motion

class Particles:
    def __init__(self,num_particles=50):
        self.util = util.Util()
        self.num_par = num_particles
        # Array of particles,
        # represent single particle as dict()
        # with "x", "y", "theta", and "weight"
        self.particle_array = [dict() for i in range(self.num_par)]
        # Empty Map
        self.grid_info = self.util.getMapInfo()

        # standard deviation of resampling
        self.res_lin_sigma = 0.3
        self.res_ang_sigma = 0.1

        # particle publisher
        self.particle_topic_name = "/particle_array"
        self.pub = rospy.Publisher(self.particle_topic_name,PoseArray,queue_size=10)
        self.pose_array = PoseArray()
        self.Initpose_array()

        # velocity noise
        self.velocity_noise = [dict() for i in range(self.num_par)]
        self.lin_vel_sigma = 0.1
        self.ang_vel_sigma = 0.05
        self.ResampleVelocityNoise()

    def ResampleAllParticles(self):
        for i in range(self.num_par):
            self.setParticle(i,self.ResampleUntilInMap())

    def ResampleUntilInMap(self):
        pose = dict()
        c = 0
        occ = True
        while  c<20:
            x = np.random.randint(0,self.grid_info.width)
            y = np.random.randint(0,self.grid_info.height)
            c+=1
            if self.util.OccupancyCheckGridCoord(x,y)==False:
                occ = False
                break
        if occ == False:
            x,y = self.util.GridPos2ActPos(x,y)
            pose["x"] = x
            pose["y"] = y
            pose["theta"]=np.random.uniform(-np.pi+0.001,np.pi)
            print("free space")
        else:
            pose["x"] = 0
            pose["y"] = 0
            pose["theta"]=np.random.uniform(-np.pi+0.001,np.pi)
            print("zero")
        pose["weight"] = 1/self.num_par
        return pose

    # robot_pose as dict() of x, y, and theta
    def ResampleAllParticlesAroundPose(self,robot_pose):
        for i in range(self.num_par):
            self.setParticle(i,self.ResampleParticleAroundPose(robot_pose,self.res_lin_sigma,self.res_ang_sigma))

    # robot_pose as dict() of x, y, and theta
    # resampling with gaussian distribution probability around pose
    def ResampleParticleAroundPose(self,robot_pose,res_lin_sigma,res_ang_sigma):
        new_par = dict()
        new_par["x"] = robot_pose["x"]+np.random.normal(0,res_lin_sigma)
        new_par["y"] = robot_pose["y"]+np.random.normal(0,res_lin_sigma)
        new_par["theta"] = robot_pose["theta"]+np.random.normal(0,res_ang_sigma)
        new_par["weight"] = 1/self.num_par
        return new_par
    
    def ResampleVelocityNoise(self):
        for i in range(self.num_par):
            self.velocity_noise[i]["linear"] = np.random.normal(0,self.lin_vel_sigma)
            self.velocity_noise[i]["angular"] = np.random.normal(0,self.ang_vel_sigma)

    def Initpose_array(self):
        self.pose_array.header.frame_id = self.util.map_frame
        self.pose_array.header.stamp = rospy.Time.now()
        self.pose_array.poses = [Pose() for i in range(self.num_par)]
        for i in range(self.num_par):
            self.pose_array.poses[i].position.z = 0.3

    def PublishParticles(self):
        poses = []
        for i in range(self.num_par):
            pose = Pose()
            pose.position.x = self.particle_array[i]["x"]
            pose.position.y = self.particle_array[i]["y"]
            qx,qy,qz,qw = quaternion_from_euler(0,0,self.particle_array[i]["theta"])
            pose.orientation.x = qx
            pose.orientation.y = qy
            pose.orientation.z = qz
            pose.orientation.w = qw
            poses.append(pose)
            self.pose_array.poses[i] = pose 
        self.pose_array.header.stamp = rospy.Time.now()
        self.pub.publish(self.pose_array)

    def getLinVelNoise(self,i):
        return self.velocity_noise[i]["linear"]

    def getAngVelNoise(self,i):
        return self.velocity_noise[i]["angular"]

    def setWeight(self,i,weight):
        self.particle_array[i]["weight"] = weight

    def getWeight(self,i):
        return self.particle_array[i]["weight"]

    def setParticle(self,i,new_par):
        self.particle_array[i] = new_par

    def setParticles(self,particle_list):
        self.particle_array = particle_list

    def getParticle(self,i):
        return self.particle_array[i]


# Test Program
if __name__=='__main__':
    rospy.init_node("particles_p")
    dt = 0.1
    rate = rospy.Rate(1/dt)

    particle_number = 50
    particles = Particles(particle_number)
    odom = rospy.wait_for_message("/odom",Odometry)

    robot_pose = dict()
    robot_pose["x"] = odom.pose.pose.position.x
    robot_pose["y"] = odom.pose.pose.position.y
    qx = odom.pose.pose.orientation.x
    qy = odom.pose.pose.orientation.y
    qz = odom.pose.pose.orientation.z
    qw = odom.pose.pose.orientation.w
    robot_pose["theta"] = euler_from_quaternion([qx,qy,qz,qw])[2]

    particles.ResampleAllParticles()

    motion = motion.MotionModel(dt)

    while not rospy.is_shutdown():
        particles.PublishParticles()
        # motion.UpdateVelocity()
        # for i in range(particle_number):
            # particle = motion.PredictMotionJoint(particles.getParticle(i))
            # particles.setParticle(i,particle)

        rate.sleep()





