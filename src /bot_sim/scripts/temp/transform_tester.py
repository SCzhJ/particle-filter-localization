#! /usr/bin/env python3
import rospy
import tf2_ros
import geometry_msgs.msg

def lookup_transform():
    rospy.init_node('tf2_lookup')

    tf_buffer = tf2_ros.Buffer(rospy.Duration(0)) # tf buffer length
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    rate = rospy.Rate(10.0) # 10Hz
    while not rospy.is_shutdown():
        try:
            trans = tf_buffer.lookup_transform('laser', 'base_link', rospy.Time())
            print("Transform from 'base_link' to 'laser':")
            print(trans)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue

        rate.sleep()

if __name__ == '__main__':
    lookup_transform()