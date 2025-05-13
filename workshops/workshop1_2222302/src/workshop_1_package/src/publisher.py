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

        # Construct publishers
        self.publisher = rospy.Publisher(
            "/counter",
            UInt8,
            #Change buff size and queue size accordingly
            queue_size=1,
        )

        self.counter = 0
        self.sendSequence = 0

        self.initialized = True
        rospy.loginfo("Node initialized!")

        
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish_time)

    def publish_time(self, event):
        if self.publisher.get_num_connections() == 0:
             rospy.loginfo("no subscriber waiting for connnection")
             time.sleep(1.0)
        msg = UInt8()
        msg.data = self.counter 
        self.publisher.publish(msg)
        self.sendSequence += 1
        rospy.loginfo("Message sent (({seqNumber})): {payload}".format(seqNumber=self.sendSequence,payload=self.counter))

        self.counter = (self.counter + 1) % 256

def on_shutdown():
    Publisher_node.timer.shutdown()  # Stops the timer explicitly, prevents messages being sent after printing amount of messages sent
    messages_send = Publisher_node.sendSequence
    rospy.loginfo("Total amount of messages sent = {}".format(messages_send))

 

if __name__ == "__main__":
    try:
        Publisher_node = Node(node_name = "Publisher_node")
        rospy.on_shutdown(on_shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
