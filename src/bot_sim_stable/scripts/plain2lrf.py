#!/usr/bin/env python3
import rospy
import tf2_ros
import tf.transformations as tf_trans
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion
import time

# def publish_transform():

#     tfBuffer = tf2_ros.Buffer()
#     listener = tf2_ros.TransformListener(tfBuffer)


#     rate = rospy.Rate(100.0)
#     while not rospy.is_shutdown():
#         try:
#             trans = tfBuffer.lookup_transform('base_link', 'front_plain_link', rospy.Time())
#             quat = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]
#             euler = tf_trans.euler_from_quaternion(quat)
#             pub.publish(euler[2])  # publish yaw angle
#             print(euler)
#         except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
#             continue

#         rate.sleep()

# if __name__ == '__main__':
#     try:
#         publish_transform()
#     except rospy.ROSInterruptException:
#         pass

from gazebo_msgs.srv import GetLinkState, GetLinkStateRequest

# 等待服务可用
rospy.init_node('plain2lrf')
pub = rospy.Publisher('plain2lrf', Float64, queue_size=10)
rospy.wait_for_service('/gazebo/get_link_state')
# 创建服务代理

get_link_srv = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)

link1 = GetLinkStateRequest()
link1.link_name = 'omni_robot::front_plain_link'  # 替换为你的link名称

link2 = GetLinkStateRequest()
link2.link_name = 'omni_robot::front_lrf_link'  # 替换为你的link名称

while not rospy.is_shutdown():
    result1 = get_link_srv(link1)
    result2 = get_link_srv(link2)
    if result1.success and result2.success:
        orientation1 = result1.link_state.pose.orientation
        orientation2 = result2.link_state.pose.orientation
        # 假设你已经获取了四元数
        quaternion1 = [orientation1.x, orientation1.y, orientation1.z, orientation1.w]
        quaternion2 = [orientation2.x, orientation2.y, orientation2.z, orientation2.w]

        # 将四元数转换为欧拉角
        euler1 = euler_from_quaternion(quaternion1)
        euler2 = euler_from_quaternion(quaternion2)
        pub.publish(euler1[2]-euler2[2])  # publish yaw angle
        print(euler1[2]-euler2[2])
    else:
        print("Failed to get link state: ", result1.status_message)