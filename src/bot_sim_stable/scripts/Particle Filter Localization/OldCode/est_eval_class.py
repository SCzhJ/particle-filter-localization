#!/usr/bin/env python3
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

'''
ROS Parameters:
    "robottame": name of the robot, default: "mycar"
'''

class EstEvalClass:
    def __init__(self):
        self.robotName = rospy.get_param("/robot_name","mycar") 
        # true robot pose and twist
        self.robotPose = Pose()
        self.robotTwist = Twist()

        # the following concise pose of 2D bot, where x, y represents x-y coordinate position, 
        # z represents angular rotation around z axis [-pi,pi]
        self.pose2D = Vector3()

        # the following will wait for gazebo
        # if gazebo not launched, they may wait forever
        self.modelStates = rospy.wait_for_message("gazebo/model_states",ModelStates)

        # the model states of gazebo contains all physical models in the simulation environment in a list. 
        # We extract the index of model we want, the robot model
        self.modelIndex = self.getModelIndex()

        # Subscribe model_states of gazebo. Put true Pose to self.robotPose. Put true Twist to self.robotTwist
        self.sub_modelStates = rospy.Subscriber("gazebo/model_states",ModelStates,self.ReadModelStates,queue_size=10)

        # Publisher of true pose
        self.poseTopicName = "/pose2D_true"
        self.pub_robotPose = rospy.Publisher(self.poseTopicName,Vector3,queue_size=10)

        # Estimation difference
        self.estDiff = Vector3()

    def getModelIndex(self):
        stateListLen = len(self.modelStates.name)
        i = 0
        while i < stateListLen:
            if self.modelStates.name[i] == self.robotName:
                index = i
            i += 1
        if index == -1:
            raise Exception("model named \'"+robotName+"\' not found, please check your model name and whether gazebo is launched")
        return index
    
    def ReadModelStates(self,msg):
        self.robotPose = msg.pose[self.modelIndex]

        self.pose2D.x = self.robotPose.position.x
        self.pose2D.y = self.robotPose.position.y

        orientationList = [self.robotPose.orientation.x,self.robotPose.orientation.y,self.robotPose.orientation.z,self.robotPose.orientation.w]
        self.pose2D.z = euler_from_quaternion(orientationList)[2]
        self.robotTwist = msg.twist[self.modelIndex]

    def PrintTruePose(self):
        print("x, y, theta")
        print(round(self.pose2D.x,5),
              round(self.pose2D.y,5),
              round(self.pose2D.z,5))
        print("")

    # hat usually represents estimation
    # _h representing hat, and z_h represents estimated bearing
    def EstDiff(self,x_h,y_h,z_h):
        self.estDiff.x = self.pose2D.x - x_h
        self.estDiff.y = self.pose2D.y - y_h
        self.estDiff.z = self.pose2D.z - z_h

    def PrintEstDiff(self):
        print("x_diff y_diff theta_diff")
        print(round(self.estDiff.x,5),
              round(self.estDiff.y,5),
              round(self.estDiff.z,5))
        print("")

class GazeboOdom:
    def __init__(self):
        self.odomTopic = "/odom"
        self.sub_odom = rospy.Subscriber(self.odomTopic,Odometry,self.RecordEstPose,queue_size=10)
        self.estPose = Pose()
        self.pose2DEst = Vector3()

    def RecordEstPose(self,msg):
        self.estPose= msg.pose.pose
        self.pose2DEst.x = self.estPose.position.x
        self.pose2DEst.y = self.estPose.position.y
        orientationList = [self.estPose.orientation.x,self.estPose.orientation.y,self.estPose.orientation.z,self.estPose.orientation.w]
        self.pose2DEst.z = euler_from_quaternion(orientationList)[2]

def receiveMsg(msg):
    for i in range(len(msg.name)):
        if msg.name[i]=
if __name__=="__main__":
    rospy.init_node("est_eval_class_p")
    rate = rospy.Rate(20)
    sub = rospy.Susbcriber("/joint_states",JointState,receiveMsg,queue_size=10)
    while not rospy.is_shutdown():
        rate.sleep()

# if __name__=="__main__":
    # rospy.init_node("est_eval_class_p")
    # estEval = EstEvalClass()
    # Odom = GazeboOdom()
    # rate = rospy.Rate(5)
    # while not rospy.is_shutdown():
        # estEval.PrintTruePose()
        # estEval.EstDiff(Odom.pose2DEst.x,Odom.pose2DEst.y,Odom.pose2DEst.z)
        # rstEval.PrintEstDiff()
        # rate.sleep()
