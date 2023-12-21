#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
import numpy as np
import pickle

class TreeNode:
    def __init__(self, index, point=Point()):
        # use the point location of the TreeNode as its unique Identification
        self.point = point
        self.index = index
        self.children_indices = []
        self.parent_index = None
    def addChildrenIndex(self, childrenIndex):
        self.children_indices.append(childrenIndex)
    def modifyParentIndex(self, newParentIndex):
        self.parent_index = newParentIndex
    def getParentIndex(self):
        return self.parent_index
    def getChildrenNum(self):
        return len(self.children_indices)
    def getChildrenIndex(self,i):
        return self.children_indices[i]
    def deleteChildrenIndex(self,i):
        self.children_indices.remove(i)

class TreeUtil:
    def __init__(self, rootPoint=Point(0,0,0)):
        self.root_node = TreeNode(0, rootPoint)
        self.tree = [self.root_node]
        self.tree_len = 1

    def getNode(self,i):
        return self.tree[i]

    def AddNode(self, point, parentIndex):
        self.tree[parentIndex].children_indices.append(self.tree_len)
        new_node = TreeNode(self.tree_len, point)
        new_node.modifyParentIndex(parentIndex)
        self.tree.append(new_node)
        self.tree_len += 1
        return self.tree_len-1

    # Input Node index and output line list of the tree attached
    def TraverseNodeAddToLineList(self, Nodei, LineList):
        Node = self.getNode(Nodei)
        child_num = Node.getChildrenNum()
        if child_num == 0:
            return LineList
        for i in range(child_num):
            child_index = Node.getChildrenIndex(i)
            LineList.append(Node.point)
            LineList.append(self.tree[child_index].point)
            LineList = self.TraverseNodeAddToLineList(child_index, LineList)
        return LineList

    def TraverseNodeAddToDebugLineList(self, Nodei, DebugLineList):
        Node = self.getNode(Nodei)
        child_num = Node.getChildrenNum()
        if child_num == 0:
            return DebugLineList
        for i in range(child_num):
            child_index = Node.getChildrenIndex(i)
            DebugLineList.append(Nodei)
            DebugLineList.append(child_index)
            DebugLineList = self.TraverseNodeAddToDebugLineList(child_index, DebugLineList)
        return DebugLineList

    def TraverseNodeAddToList(self, Nodei=0, List=[]):
        Node = self.getNode(Nodei)
        List.append(Node)
        child_num = Node.getChildrenNum()
        if child_num == 0:
            return List
        for i in range(child_num):
            child_index = Node.getChildrenIndex(i)
            List.append(self.tree[child_index].point)
            List = self.TraverseNodeAddToList(child_index, List)
        return List

class MapUtil:
    def __init__(self,occ_Threshold=95,CostMapFile="CostMap/CostMapR0d5"):
        self.map_frame = "map"
        self.odom_frame = "odom"
        self.grid_map = None
        self.grid_info = None
        self.cost_map = None
        self.LoadCostMap(CostMapFile)
        self.LoadMap()
        # map occupancy probability from 0 to 100, set a threshold to determine occupancy
        self.occ_threshold = occ_Threshold

    def LoadCostMap(self,CostMapFile):
        with open(CostMapFile,'rb') as f:
            self.cost_map = pickle.load(f)

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

    # check occupancy in cost map inputing grid coordinate
    def OccupancyCheckCostMapGridCoord(self,x,y):
        if x > self.grid_info.width or y > self.grid_info.height or x < 0 or y < 0:
            return True
        occ = self.cost_map[y*self.grid_info.width+x]
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

if __name__=="__main__":
    rospy.init_node("tree_marker_util_test_p")
    rate = rospy.Rate(10)
    marker_pub = rospy.Publisher('tree_marker', Marker, queue_size=10)

    Tree = TreeUtil(rootPoint=Point(0,0,0))
    nodei_1 = Tree.AddNode(Point(1,2,0), 0)
    nodei_2 = Tree.AddNode(Point(1,-3,0), 0)
    nodei_3 = Tree.AddNode(Point(3,3,0), 0)
    nodei_4 = Tree.AddNode(Point(3,1,0), 0)

    nodei_5 = Tree.AddNode(Point(5,4,0),nodei_3)

    LineList = []
    LineList = Tree.TraverseNodeAddToLineList(0,LineList)
    print(LineList)

    while not rospy.is_shutdown():
        marker_id = 0
    
        # Create the root point
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "marker_tree"
        
        marker.id = marker_id
        marker_id += 1
    
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime = rospy.Duration()
    
        marker.points = LineList
        marker_pub.publish(marker)
        rate.sleep()

