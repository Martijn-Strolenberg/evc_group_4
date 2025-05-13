#!/usr/bin/env python2

import rospy
from std_msgs.msg import UInt8
import time


class PublishNode:
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

        self.sendSequence = 0

        self.initialized = True
        rospy.loginfo("Node initialized!")
        self.frequency = 10.0 # Hz
        
        # wait for subscribers to connect
        while self.publisher.get_num_connections() == 0:
            rospy.loginfo("Waiting for subscribers to connect...")
            time.sleep(1.0)
        rospy.loginfo("Subscribers connected!")

        self.timer = rospy.Timer(rospy.Duration(1.0/self.frequency), self.publish_time)


    def publish_time(self, event):
        msg = UInt8()
        msg.data = self.sendSequence % 256  # wrap around at 256

        self.publisher.publish(msg)

        rospy.loginfo("Message sent (({seqNumber})): {payload}".format(seqNumber=self.sendSequence,payload=msg))
        self.sendSequence += 1



if __name__ == "__main__":
    try:
        publish_node = PublishNode(node_name = "publish_node")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
