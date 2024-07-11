#!/usr/bin/env python3
import rospy
import tf2_ros

def handle_transform(transform):
    # This function will be called when a new transform is published
    print("Received transform: ", transform)

def tf_subscriber():
    rospy.init_node('tf_subscriber')

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            trans = tf_buffer.lookup_transform('odom', 'base_footprint', rospy.Time())
            handle_transform(trans)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        rate.sleep()

if __name__ == '__main__':
    tf_subscriber()