#!/usr/bin/env python2

import rospy
from std_msgs.msg import UInt8
import time


class SubscriberNode:
    def __init__(self, node_name):
        self.initialized = False
        self.node_name = node_name
        rospy.loginfo("Initializing ROS Subscribe node...")
        rospy.init_node(self.node_name, anonymous=True)

        # Construct subscribers
        self.subscriber = rospy.Subscriber(
            "/counter",
            UInt8,
            self.subscriber_cb,
            #Change buff size and queue size accordingly
            buff_size=100000000,
            queue_size=10,
        )

        self.publishConcurrentMisPredict = True
        self.concurrentMisPredict = 0

        if self.publishConcurrentMisPredict:
            # Construct publisher
            self.publisher = rospy.Publisher(
                "/mis_predict",
                UInt8,
                queue_size=10,
            )

        self.recvSequence = 0
        self.artficialDelay = 1 # seconds

        self.initialized = True
        rospy.loginfo("Node initialized!")


    def subscriber_cb(self, data):
        if not self.initialized:
            return
        
        # check if the data is equal to received sequence number
        expected = self.recvSequence % 256
        received = data.data
        if received != expected:
            rospy.logwarn("Received sequence number does not match expected value, (expected: {expected}, received: {received})".format(expected=expected, received=received))
            # publish the mis-predict message
            if self.publishConcurrentMisPredict:
                self.concurrentMisPredict += 1
                self.publisher.publish(expected)
                rospy.loginfo("Published mis-predict message: {received}".format(received=received))
            
        else:
            # log the received message
            if self.publishConcurrentMisPredict:
                # publish the mis-predict message
                self.publisher.publish(0)
                rospy.loginfo("Published mis-predict message: 0")
                # reset the concurrent mis-predict counter
                self.concurrentMisPredict = 0
            
            rospy.loginfo("Received message ({seqNumber}): {payload}".format(seqNumber=self.recvSequence,payload=received))

        self.recvSequence += 1

        # Artificially slow down the subscriber
        rospy.sleep(self.artficialDelay)



if __name__ == "__main__":
    try:
        subscriber_node = SubscriberNode(node_name = "subscriber_node")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
