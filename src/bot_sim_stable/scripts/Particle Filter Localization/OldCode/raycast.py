#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from geometry_msgs.msg import Pose, PoseStamped
import numpy as np
import drifter_class as dft
import est_eval_class as est
import util

class Raycast:
    def __init__(self):

        self.util = util.Util()
        # subscribe to scan
        self.numScanTopic = 0
        self.subLaser = rospy.Subscriber("/scan",LaserScan,self.RcvScan,queue_size=10)
        self.scan = LaserScan()

        # prepare publication for raycast
        self.pubRaycast = rospy.Publisher("/scan_raycast",LaserScan,queue_size=10)
        self.raycastScan = LaserScan()
        self.scan_num = 0

        # settle transform for our raycast
        self.laserFrame = "laser"
        self.laserSupportFrame = "support"
        self.raycastFrame = "raycast"
        self.drifter = dft.Drifter()
        self.rcTrans = self.GetRCTrans()

        # prepare map
        self.originRelativePose = Pose()
        self.gridMap = self.util.LoadMap()
        self.resolution = self.gridMap.info.resolution
        self.width = self.gridMap.info.width
        self.height = self.gridMap.info.height

        # prepare raycast calculation
        self.rays = [-1]

        # raycast location
        self.pubRayOrigin = rospy.Publisher("/rayorigin_pose",PoseStamped,queue_size=10)

    def getRanges(self):
        return self.scan.ranges

    def getRays(self):
        return self.rays

    # pass in: robot pose Vector3 (x,y,theta)
    # return a list of measurement that is expected
    # sample 1 per sampleNum
    def RaycastDDA(self,robotPose,sampleNum):
        self.rays = []

        # i'm to lazy to calculate the rotational transform of map origin
        # since in this example, it is zero, we will ignore it
        robotPose[0] -= self.originRelativePose.position.x
        robotPose[1] -= self.originRelativePose.position.y
        robotPose[0] = robotPose[0]/self.resolution
        robotPose[1] = robotPose[1]/self.resolution

        angle = self.scan.angle_min
        
        for i in range(len(self.scan.ranges)):
            if i % sampleNum == 0:
                realAngle = robotPose[2] + angle
                rayDir = [np.cos(realAngle),np.sin(realAngle)]

                rayUnitStep = [
                    np.sqrt(1+(rayDir[1]/rayDir[0])**2),
                    np.sqrt(1+(rayDir[0]/rayDir[1])**2)
                    ]

                mapCheck = [
                    int(robotPose[0]),
                    int(robotPose[1])
                    ]

                rayLength = [0,0]
                step = [0,0]

                if rayDir[0] < 0:
                    step[0] = -1
                    rayLength[0] = (robotPose[0]-mapCheck[0])*rayUnitStep[0]
                else:
                    step[0] = 1
                    rayLength[0] = ((mapCheck[0]+1)-robotPose[0])*rayUnitStep[0]

                if rayDir[1] < 0:
                    step[1] = -1
                    rayLength[1] = (robotPose[1]-mapCheck[1])*rayUnitStep[1]
                else:
                    step[1] = 1
                    rayLength[1] = ((mapCheck[1]+1)-robotPose[1])*rayUnitStep[1]

                boundFound = False
                longestRay = 0

                while not boundFound:
                    if rayLength[0] < rayLength[1]:
                        mapCheck[0] += step[0]
                        longestRay = rayLength[0]
                        rayLength[0] += rayUnitStep[0]
                    else:
                        mapCheck[1] += step[1]
                        longestRay = rayLength[1]
                        rayLength[1] += rayUnitStep[1]
                    # map check here
                    if self.util.OccupancyCheckGridCoord(mapCheck[0],mapCheck[1]):
                        boundFound = True
                # turn ray in cell unit to m
                self.rays.append(longestRay * self.resolution)

            angle += self.scan.angle_increment

        return self.rays

    def RcvScan(self,msg):
        self.scan = msg
        self.scan_num = len(msg.ranges)
        self.numScanTopic += 1

    # Test functions
    def PublishRCandRCtrans(self):
        self.drifter.RepeatedTransform(self.laserSupportFrame,self.raycastFrame)
        self.raycastScan = self.scan
        self.raycastScan.header.frame_id = self.raycastFrame
        self.raycastScan.header.stamp = rospy.Time.now()
        self.raycastScan.ranges = self.rays
        self.pubRaycast.publish(self.raycastScan)

    def GetRCTrans(self):
        self.drifter.getTransform(self.laserSupportFrame,self.laserFrame)
        return self.drifter.rcvTrans

# Test Program
if __name__ == '__main__':
    rospy.init_node("raycast_p")
    rc = Raycast()
    rate = rospy.Rate(40)
    run_num = 1
    odometry = est.GazeboOdom()
    while not rospy.is_shutdown() and run_num != 0:
        if rc.numScanTopic > 0:
            rc.RaycastDDA([
                odometry.pose2DEst.x,
                odometry.pose2DEst.y,
                odometry.pose2DEst.z],1)
            rc.PublishRCandRCtrans()
            rc.numScanTopic -= 1
        rate.sleep()
