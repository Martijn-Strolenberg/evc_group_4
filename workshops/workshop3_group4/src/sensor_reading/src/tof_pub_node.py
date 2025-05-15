#!/usr/bin/env python2

import rospy
import numpy as np
from tofDriver import VL53L0X
from std_msgs.msg import Float64


class TofPublisherNode:
    def __init__(self):
        self.initialized = False
        rospy.loginfo("Initializing tof publisher node...")

        # Init Publisher
        self.pub_tof = rospy.Publisher(
            "/tof",
            Float64,
            queue_size=10
        )
        self.sensor = VL53L0X()
        rospy.loginfo("VL53L0X initialized.")


    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        try:
            while not rospy.is_shutdown():
                try:
                    distance = self.sensor.read_distance()
                    self.pub_tof.publish(distance)
                except rospy.ROSInterruptException:
                    break

                rate.sleep()
        except (rospy.ROSInterruptException, KeyboardInterrupt):
            rospy.loginfo("Shutting down tof publisher.")
    

# Main function run the node
if __name__ == "__main__":
    # Initialize the nodes
    rospy.init_node('tof_pub_node')
    tof_node = TofPublisherNode()
    tof_node.run()
