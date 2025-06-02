#!/usr/bin/env python2

import rospy
from std_msgs.msg import Float64

class CollisionDetectionNode:
 
    def __init__(self):
        self.initialized = False
        rospy.loginfo("Initializing Collision Detection node...")

        # Make a subscriber to ToF sensor
        self.sub_tof = rospy.Subscriber(
            "/tof",
            Float64,
            self.set_tof_cb,
            buff_size=2**24,
            queue_size=10
        )
        self.pub_tof

        self.initialized = True
        rospy.loginfo("Collision Detection node initialized!")

    
    def set_tof_cb(self, msg):
        self.tof = msg.data
        rospy.loginfo("TOF distance: %f", self.tof)

        


if __name__ == "__main__":
    # Initialize the node
    rospy.init_node('collision_detection_node')
    collision_detection_node = CollisionDetectionNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down collision detection node.")