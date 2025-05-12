#!/usr/bin/env python2

import rospy
from std_msgs.msg import UInt8
from assign1_package.msg import test1
import time


class counter:
    def __init__(self):
        rospy.loginfo("Initalisation")
        
        # To send indexes
        self.int8subscribe = 0

        # Initalisation of node
        rospy.init_node("node", anonymous=True)

        #init sub and pub
        self.recieved = rospy.Subscriber("counter", test1, self.reciever,queue_size = 10) # queue_size changed for question 4 
        self.incoming = rospy.Publisher("incoming",test1,queue_size = 1) # temporary publisher for plotting using rqt_plot
        self.estimated  = rospy.Publisher("estimated",test1,queue_size = 1) # temporary publisher for plotting using rqt_plot

    def reciever(self,data): ## The subscriber
        recieved_data = data.test

        if self.int8subscribe != recieved_data: ## Warning when the estimator and data are not the same
            rospy.loginfo("Warning!, Recieved data: {pub}, " \
                          " and Subscriber step: {sub} are not the same. Time: {tim} ".format(pub = recieved_data,
                                                                             sub = self.int8subscribe, tim = time.time()))
        else: ## Message prompt when correct
            rospy.loginfo("Recieved data: {pub}, " \
                          " and Subscriber step: {sub} are the same. Time: {tim}".format(pub = recieved_data,
                                                                                    sub = self.int8subscribe, tim = time.time()))
            
        msg1 = test1()
        msg2 = test1()

        msg1.test = self.int8subscribe
        self.estimated.publish(msg1) # publish the estimated message for comparison

        msg2.test = recieved_data
        self.incoming.publish(msg2) # publish the actual incoming message

        # reset estimator if it exceeds the value allowed by UInt8
        if(self.int8subscribe >= 255):
            self.int8subscribe = 0
        else:
            self.int8subscribe += 1

        rospy.sleep(1.0) # 1 hz ~ 1 second #### This is commented out before question 3 of the assignment

if __name__ == "__main__":
    try:
        thing = counter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 