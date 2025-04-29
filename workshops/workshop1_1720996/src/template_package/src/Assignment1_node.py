#!/usr/bin/env python2

import rospy
from std_msgs.msg import Float64,UInt8
import time


""" class Node:
    def __init__(self, node_name):
        self.initialized = False
        self.node_name = node_name
        rospy.loginfo("Initializing ROS node...")
        rospy.init_node(self.node_name, anonymous=True)

        # Construct subscribers
        self.subscriber = rospy.Subscriber(
            "/endpoint",
            Float64,
            self.subscriber_cb,
            #Change buff size and queue size accordingly
            buff_size=1000000,
            queue_size=1,
        )

        # Construct publishers
        self.publisher = rospy.Publisher(
            "/endpoint",
            Float64,
            #Change buff size and queue size accordingly
            queue_size=1,
        )

        self.recvSequence = 0
        self.sendSequence = 0

        self.initialized = True
        rospy.loginfo("Node initialized!")
        self.timer = rospy.Timer(rospy.Duration(1.0), self.publish_time)


    def subscriber_cb(self, data):
        if not self.initialized:
            return
        
        self.recvSequence += 1
        rospy.loginfo("Received message ({seqNumber}): {payload}".format(seqNumber=self.recvSequence,payload=data.data))
    
    def publish_time(self, event):
        msg = Float64()
        msg.data = time.time()

        self.publisher.publish(msg)
        self.sendSequence += 1
        
        rospy.loginfo("Message sent (({seqNumber})): {payload}".format(seqNumber=self.sendSequence,payload=msg))
 """
""" if __name__ == "__main__":
    try:
        camera_node = Node(node_name = "template_node")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass """


class counter:
    def __init__(self):
        rospy.loginfo("Initalisation")

        self.int8publish = 0
        self.int8subscribe = 0

        rospy.init_node("UInt8_counter", anonymous=True)

        self.recieved = rospy.Subscriber("\counter", UInt8, self.callback,queue_size = 1)
        self.send = rospy.Publisher("\counter",UInt8,queue_size = 1)

    def callback(self,data):
        recieved_data = data.data
        if self.int8subscribe != recieved_data:
            rospy.loginfo(f"Warning, Recieved data: {recieved_data}, Subscriber step: {self.int8subscribe} are not the same ")
        else:
            rospy.loginfo(f"Recieved data: {recieved_data}, Subscriber step: {self.int8subscribe} are the same")

        if(self.int8subscribe <= 255):
            self.int8subscribe = 0
        else:
            self.int8subscribe += 1

    
    def sender(self):
        publish_rate = rospy.Rate(1)
        msg = UInt8()
        msg.data = self.int8publish

        if(self.int8publish <= 255):
            self.int8publish = 0
        else:
            self.int8publish += 1

        self.send.publish(msg)
        publish_rate.sleep()

""" def counter():

    rospy.loginfo("Initalisation")

    rospy.init_node("UInt8_counter", anonymous=True)
    rospy.Subscriber("/count_Luka",UInt8,reciever)

    rospy.spin() """



if __name__ == "__main__":
    try:
        counter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 