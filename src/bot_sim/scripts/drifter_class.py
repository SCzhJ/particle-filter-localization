#!/usr/bin/env python3

'''
the class is for Odometry Drift and Transform Drifts
'''

import rospy
import math
import tf2_ros
import geometry_msgs.msg
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class Drifter:
    def __init__(self,From="map",To="odom"):
        # tf listener
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # tf broadcaster
        self.tfBr = tf2_ros.TransformBroadcaster()

        self.staticTfBr= tf2_ros.StaticTransformBroadcaster()

        # defining tranform From -> To
        self.transFrom = From
        self.transTo = To

        # translation and rotation's x y z default set to 0.0
        self.translation = geometry_msgs.msg.Vector3()
        self.rotation = geometry_msgs.msg.Vector3()

        # Set the transform by above translation and rotation
        self.myTrans = geometry_msgs.msg.TransformStamped()

        # Received transform
        self.rcvTrans = geometry_msgs.msg.TransformStamped()


    # the following calculate new translation from current translation 
    # So that the robot could be on the Pose we want
    ### def DriftOnThisPosition(self,Pose,CurrTrans)
        

    # because the odometry in simulation is really accurate,
    # we can treat it as the true pose of the robot,
    # adding no transform between map and odom
    def RepeatedTransform(self,FromName,ToName):
        self.myTrans = self.rcvTrans
        self.myTrans.header.frame_id = FromName
        self.myTrans.header.stamp = rospy.Time.now()
        self.myTrans.child_frame_id = ToName
        self.tfBr.sendTransform(self.myTrans)

    def ZeroDrift(self):
        self.translation.x = 0.0
        self.translation.y = 0.0
        self.translation.z = 0.0
        self.rotation.x = 0.0
        self.rotation.y = 0.0
        self.rotation.z = 0.0
        self.SetTransform()
        self.staticTfBr.sendTransform(self.myTrans)

    def SetTransform(self):
        self.myTrans.header.stamp = rospy.Time.now()
        self.myTrans.header.frame_id = self.transFrom#don't change, not typo
        self.myTrans.child_frame_id = self.transTo
        self.myTrans.transform.translation.x = self.translation.x
        self.myTrans.transform.translation.y = self.translation.y
        self.myTrans.transform.translation.z = self.translation.z
        qx,qy,qz,qw = quaternion_from_euler(self.rotation.x,self.rotation.y,self.rotation.z)
        self.myTrans.transform.rotation.x = qx
        self.myTrans.transform.rotation.y = qy
        self.myTrans.transform.rotation.z = qz
        self.myTrans.transform.rotation.w = qw

    def getTransform(self,FromThisFrame,ToThisFrame):
        get = False
        rate = rospy.Rate(20)
        while (not rospy.is_shutdown()) and get == False:
            try:
                self.rcvTrans = self.tfBuffer.lookup_transform(FromThisFrame,ToThisFrame,rospy.Time())
                get = True
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.loginfo("not getting tf, trying to get again")
                continue
            rate.sleep()

if __name__=="__main__":
    rospy.init_node("drifter_p")
    drifter = Drifter()
    rate = rospy.Rate(10)
    drifter.ZeroDrift()
    while not rospy.is_shutdown():
        rate.sleep()
