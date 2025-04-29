#!/usr/bin/env python2

import rospy
from std_msgs.msg import UInt8
import time


class Node:
    def __init__(self, node_name):
        self.initialized = False
        self.node_name = node_name
        rospy.loginfo("Initializing publishing ROS node...")
        rospy.init_node(self.node_name, anonymous=True)

        # Construct publishers
        self.publisher = rospy.Publisher(
            "/counter",
            UInt8,
            #Change buff size and queue size accordingly
            queue_size=1,
        )

        self.sendSequence = 0
        self.counter = 0

        self.initialized = True
        rospy.loginfo("Node initialized!")
        self.timer = rospy.Timer(rospy.Duration(0.10), self.publish_time) # publishing 10 Hz

    
    def publish_time(self, event):
        msg = UInt8()
        msg.data = self.counter

        if (self.counter == 255):
            self.counter = 0
        else:
            self.counter += 1
        
        self.publisher.publish(msg)
        self.sendSequence += 1
        
        rospy.loginfo("Message sent[{seqNumber}]: {payload}".format(seqNumber=self.sendSequence,payload=msg))

if __name__ == "__main__":
    try:
        camera_node = Node(node_name = "pub_node")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
