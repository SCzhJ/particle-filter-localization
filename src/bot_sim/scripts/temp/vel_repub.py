#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

class VelocityPublisher:
    def __init__(self, publish_rate):
        self.publish_rate = publish_rate
        self.last_received_cmd = Twist()

        # Initialize the node
        rospy.init_node('velocity_republisher')

        # Create a subscriber to the cmd_vel topic
        self.subscriber = rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback)

        # Create a publisher to republish the cmd_vel at a given rate
        self.publisher = rospy.Publisher("velocity", Twist, queue_size=10)

        # Create a Rate object to control the publish rate
        self.rate = rospy.Rate(publish_rate)

    def cmd_vel_callback(self, msg):
        """Callback function for the cmd_vel subscriber."""
        self.last_received_cmd = msg

    def run(self):
        """Main loop to republish the velocity command at the set rate."""
        while not rospy.is_shutdown():
            self.publisher.publish(self.last_received_cmd)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        # Set the rate of republishing. For example, 10 Hz.
        publish_rate = 10  # Hz

        velocity_publisher = VelocityPublisher(publish_rate)
        velocity_publisher.run()
        
    except rospy.ROSInterruptException:
        pass
