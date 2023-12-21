#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
import numpy as np

class Util:
    def __init__(self,occ_Threshold=95):
        self.map_frame = "map"
        self.odom_frame = "odom"
        self.grid_map = None
        self.grid_info = None
        self.LoadMap()
        # map occupancy probability from 0 to 100, set a threshold to determine occupancy
        self.occ_threshold = occ_Threshold

    # take world coordinate, return grid pixel position
    def ActPos2GridPos(self,x,y):
        x -= self.grid_info.origin.position.x
        y -= self.grid_info.origin.position.y
        x = int(x/self.grid_info.resolution)
        y = int(y/self.grid_info.resolution)
        return x,y

    # take grid pixel position, return world coordinate
    def GridPos2ActPos(self,x,y):
        x *= self.grid_info.resolution
        y *= self.grid_info.resolution
        x += self.grid_info.origin.position.x
        y += self.grid_info.origin.position.y
        return x,y

    # check occupancy inputing grid coordinate
    def OccupancyCheckGridCoord(self,x,y):
        if x > self.grid_info.width or y > self.grid_info.height or x < 0 or y < 0:
            return True
        occ = self.grid_map[y*self.grid_info.width+x]
        if occ == -1 or occ>self.occ_threshold:
            return True
        else:
            return False

    # return 1D grid map
    def getMap(self):
        return self.grid_map

    def getMapInfo(self):
        return self.grid_info

    def LoadMap(self):
        rospy.wait_for_service('/static_map') 
        gridMapService = rospy.ServiceProxy('/static_map',GetMap)
        response = gridMapService()
        print("map loaded")
        self.grid_map = response.map.data
        self.grid_info = response.map.info

    def WrapToPosNegPi(self,theta):
        while theta > np.pi:
            theta -= 2*np.pi
        while theta < -np.pi:
            theta += 2*np.pi
        return theta


# Test Program for Util

# change rviz map topic to "modified_map"
# change rviz pose topic to "pose_c"
# use rviz published point to click a point
# and test the utility of Util

util = Util()
pose_c = PoseStamped()
pose_c.header.frame_id = "map"
modified_grid = list(util.getMap())
clicked = False
# Callback function for the rviz click event
def clickCallback(msg):
    global util
    global modified_grid
    global clicked
    global pose_c
    print("clicked called")
    # Convert rviz position to occupancy grid position
    grid_x, grid_y = util.ActPos2GridPos(msg.point.x, msg.point.y)
    pose_c.pose.position.x,pose_c.pose.position.y = util.GridPos2ActPos(grid_x,grid_y)
    print(pose_c.pose.position.x,pose_c.pose.position.y)

    # Set the occupancy in the surrounding area to 100 (occupied)
    for dx in range(-20, 20):
        for dy in range(-20, 20):
            x = grid_x + dx
            y = grid_y + dy
            if not util.OccupancyCheckGridCoord(x, y):
                modified_grid[y * util.grid_info.width + x] = 100
    clicked = True

if __name__=="__main__":
    rospy.init_node("util_test_p")
    rate = rospy.Rate(10)
    # Create a publisher to publish the modified grid map
    pub = rospy.Publisher("/modified_map", OccupancyGrid, queue_size=10)
    pub_pose = rospy.Publisher("/pose_c", PoseStamped, queue_size=10)

    # Create a subscriber to listen to the rviz click event
    rospy.Subscriber("/clicked_point", PointStamped, clickCallback,queue_size=10)

    occupancy_grid = OccupancyGrid()
    occupancy_grid.header.frame_id = "map"
    occupancy_grid.info = util.grid_info

    print("enter loop")

    while not rospy.is_shutdown():
        if clicked == True:
            clicked = False
            print("clicked")
        occupancy_grid.header.stamp = rospy.Time.now()
        occupancy_grid.data = modified_grid
        pub.publish(occupancy_grid)

        pose_c.header.stamp = rospy.Time.now()
        pub_pose.publish(pose_c)

        rate.sleep()

