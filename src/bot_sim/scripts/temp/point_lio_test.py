#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

def callback(msg):
    # 获取四元数
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w
    )

    # 将四元数转换为欧拉角
    euler = euler_from_quaternion(quaternion)

    # 打印角度
    print('Yaw: {}'.format(euler[2]))
    print('Pose: {}'.format(msg.pose.pose.position))

def main():
    rospy.init_node('odom_listener')

    # 订阅odom主题
    rospy.Subscriber("/aft_mapped_to_init", Odometry, callback)
    # 保持监听直到节点被关闭
    rospy.spin()

if __name__ == '__main__':
    main()