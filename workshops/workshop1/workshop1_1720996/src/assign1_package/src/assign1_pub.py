#!/usr/bin/env python3


import rospy
from std_msgs.msg import UInt8
from assign1_package.msg import test1
import time


class counter:
    def __init__(self):
        rospy.loginfo("Initalisation")
        
        # To send indexes
        self.int8publish = 0

        # Initalisation of node
        rate = 10.0 # For question 1 this was changed to 1.0
        rospy.init_node("node", anonymous=True)

        #init pub 
        self.send = rospy.Publisher("counter",test1,queue_size = 1)

        self.timer = rospy.Timer(rospy.Duration(1.0/rate), self.sender) # to automatically run publisher

    
    def sender(self,event): ## The publisher
        
        msg = test1()
        msg.test = self.int8publish

        self.send.publish(msg)

        # reset data if it exceeds alloted space of UInt8
        if(self.int8publish >= 255):
            self.int8publish = 0
        else:
            self.int8publish += 1
        

if __name__ == "__main__":
    try:
        thing = counter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 