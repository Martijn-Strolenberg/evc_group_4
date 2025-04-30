#!/usr/bin/env python2

import rospy
from std_msgs.msg import Float64, UInt8
import time


class Node:
    def __init__(self, node_name):
        self.initialized = False
        self.node_name = node_name
        rospy.loginfo("Initializing ROS node...")
        rospy.init_node(self.node_name, anonymous=True)

        # Construct subscribers
        self.subscriber = rospy.Subscriber(
            "/endpoint",
            UInt8,
            self.subscriber_cb,
            #Change buff size and queue size accordingly
            buff_size=1000000,
            queue_size=10,
        )

        # Construct publishers
        self.publisher = rospy.Publisher(
            "/endpoint",
            UInt8,
            #Change buff size and queue size accordingly
            queue_size=1,
        )

        self.recvSequence = 0
        self.sendSequence = 0
        self.incrementing_number = 0
        self.prediction_nxt = 0

        self.initialized = True
        rospy.loginfo("Node initialized!")
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish_time)


    def subscriber_cb(self, data):
        if not self.initialized:
            return
        
        self.recvSequence += 1

        self.prediction_nxt = ( self.recvSequence - 1 ) % 256

        time.sleep(1)
            
        if(self.prediction_nxt != data.data):
            # Error
            rospy.loginfo("RECEIVER ERROR [{seqNumber}]: {payload} PREDICTED {prediction}".format(seqNumber=self.recvSequence,
                                                                                                  payload=data.data, prediction=self.prediction_nxt))
        else:
            # No error
            rospy.loginfo("RECEIVER [{seqNumber}]: {payload} PREDICTED {prediction}".format(seqNumber=self.recvSequence,
                                                                                                  payload=data.data, prediction=self.prediction_nxt))
    
    def publish_time(self, event):
        msg = UInt8()
        msg.data = self.incrementing_number

        if(self.incrementing_number == 255): 
            self.incrementing_number = 0
        else:
            self.incrementing_number = self.incrementing_number + 1;

        self.publisher.publish(msg)
        self.sendSequence += 1
        
        rospy.loginfo("PUBLISH [{seqNumber}] {payload}".format(seqNumber=self.sendSequence,payload=msg))

if __name__ == "__main__":
    try:
        camera_node = Node(node_name = "template_node")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
