#!/usr/bin/env python2

import rospy
from std_msgs.msg import UInt8
import time


class Node:
    def __init__(self, node_name):
        self.initialized = False
        self.node_name = node_name
        rospy.loginfo("Initializing subscribing ROS node...")
        rospy.init_node(self.node_name, anonymous=True)

        # Construct subscribers
        self.subscriber = rospy.Subscriber(
            "/counter",
            UInt8,
            self.subscriber_cb,
            #Change buff size and queue size accordingly
            buff_size=1000000,
            queue_size=1,
        )

        self.recvSequence = 0
        self.expected_data = None

        self.initialized = True
        rospy.loginfo("Node initialized!")


    def subscriber_cb(self, data):
        time.sleep(1) # Simulate some processing time
        if not self.initialized:
            return   
        
        if self.expected_data is None:
            if data.data != 255:
                self.expected_data = data.data + 1
            else: 
                self.expected_data = 0
        else:
            if self.expected_data == 255:
                self.expected_data = 0
            else:
                self.expected_data += 1
        
        self.recvSequence += 1
        
        exp_val = self.expected_data - 1
        if exp_val == -1:
            exp_val = 255

        rospy.loginfo("Received message[{seqNumber}] value: {payload} expected: {expected}".format(seqNumber=self.recvSequence, payload=data.data, expected=exp_val))
        if data.data != exp_val:
            rospy.logerr("Received unexpected data message[{seqNumber}]: {payload} but expected value {expected}".format(seqNumber=self.recvSequence, payload=data.data, expected=exp_val))
    

if __name__ == "__main__":
    try:
        camera_node = Node(node_name = "sub_node")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
