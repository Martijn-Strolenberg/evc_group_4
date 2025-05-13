#!/usr/bin/env python2

import rospy
from std_msgs.msg import UInt8
import time


class Node:
    def __init__(self, node_name):
        self.initialized = False
        self.node_name = node_name
        rospy.loginfo("Initializing ROS node...")
        rospy.init_node(self.node_name, anonymous=True)

        # Construct subscribers
        self.subscriber = rospy.Subscriber(
            "/counter",
            UInt8,
            self.subscriber_cb,
            #Change buff size and queue size accordingly
            buff_size=1000000,
            queue_size=10,
        )


        # Create a publisher for the received value only added for part 4
        self.expected_publisher = rospy.Publisher("/received_value", UInt8, queue_size=1)

        self.recvSequence = 0
        self.incorrect_messages = 0
        self.last_received = None
        # self.expected_value = 0
        self.initialized = True
        rospy.loginfo("Node initialized!")


    #subscriber callback
    def subscriber_cb(self, data):
        # added delay for 1hz 
        rospy.sleep(1)

        if not self.initialized:
            return
        
        current_data = data.data
        self.recvSequence += 1 

        # Publish received value (added for part 4 visualizing received values)
        received_msg = UInt8()
        received_msg.data = current_data
        self.expected_publisher.publish(received_msg)

        # Correct & wrong message cases
        if self.last_received is not None: 
            expected_data = (self.last_received + 1)%256

            if expected_data == current_data:
                rospy.loginfo("Correct received message ({seqNumber}): {payload} expected value = {expected} ".format(seqNumber=self.recvSequence,payload=current_data, expected=expected_data))
            else:
                rospy.logwarn("Warning message wrong ({seqNumber}):received: {payload} expected:{expected}".format(seqNumber=self.recvSequence,payload=current_data,expected=expected_data))
                self.incorrect_messages += 1
        # First message case
        else:
            rospy.loginfo("First message ({seqNumber}): {payload}".format(seqNumber=self.recvSequence,payload=current_data))

        # set last received data to current received data (used to sync data if its out of sync)
        self.last_received = current_data
      
def on_shutdown():
    total = subscriber_node.recvSequence
    incorrect = subscriber_node.incorrect_messages
    #print statistics 
    rospy.loginfo("Total amount of messages received = {}, incorrect = {}".format(total,incorrect))
    
if __name__ == "__main__":
    try:
        subscriber_node = Node(node_name = "subscriber_node")
        rospy.on_shutdown(on_shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass