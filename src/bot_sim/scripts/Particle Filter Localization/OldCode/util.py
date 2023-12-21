#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap


class Util:
    def __init__(self,occThreshold=95):
        self.gridMap = self.LoadMap()
        # map occupancy probability from 0 to 100, set a threshold to determine occupancy
        self.occThreshold = occThreshold

    # take world coordinate, return grid pixel position
    def ActPos2GridPos(self,x,y):
        x -= self.gridMap.info.origin.position.x
        y -= self.gridMap.info.origin.position.y
        x = int(x/self.gridMap.info.resolution)
        y = int(y/self.gridMap.info.resolution)
        return x,y

    # take grid pixel position, return world coordinate
    def GridPos2ActPos(self,x,y):
        x *= self.gridMap.info.resolution
        y *= self.gridMap.info.resolution
        x += self.gridMap.info.origin.position.x
        y += self.gridMap.info.origin.position.y
        return x,y

    def OccupancyCheckGridCoord(self,x,y):
        if x > self.gridMap.info.width or y > self.gridMap.info.height or x < 0 or y < 0:
            return True
        occ = self.gridMap.data[y*self.gridMap.info.width+x]
        if occ == -1 or occ>self.occThreshold:
            return True
        else:
            return False

    def getMap(self):
        return self.gridMap

    def LoadMap(self):
        rospy.wait_for_service('/static_map') 
        gridMapService = rospy.ServiceProxy('/static_map',GetMap)
        response = gridMapService()
        self.originRelativePose = response.map.info.origin
        print("map loaded")
        return response.map
