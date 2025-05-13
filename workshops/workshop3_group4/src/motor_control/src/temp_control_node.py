#!/usr/bin/env python2



import time

import hat

from hat import *

import motorDriver

from motorDriver import DaguWheelsDriver

import cv2

import rospy

import numpy as np



from sensor_msgs.msg import CompressedImage

from jetson_camera.msg import twovids

from motor_control.msg import motor_cmd,encoder





class MotorSubscriberNode:

    def __init__(self):

        self.initialized = False

        rospy.loginfo("Initializing motor control node...")





        # Construct subscriber

        self.sub_cmd = rospy.Subscriber(

            "/encoder",

            encoder,

            self.motor_cb,

            buff_size=2**24,

            queue_size=10

        )

        self.gain, self.trim = self.load_param()

        self.initialized = True
        self.prev_mesg = 0

        rospy.loginfo("motor control node initialized!")





    def motor_cb(self, data):

        if not self.initialized:

            return

        distance = data.abs_distance 
        angle  = data.abs_angle
        new_mesg = data.new_mesg
        if new_mesg != self.prev_mesg:
            motor = DaguWheelsDriver() # initialize motor drivers

            # step 1 turn the robot to the required angle
            if angle > 0:
                #motor.set_wheels_speed(left=-(self.gain - self.trim)*0.1, right=(self.gain + self.trim)*0.1) # GO LEFT!
                motor.set_wheels_speed(left=(self.gain - self.trim)*0.15, right=-(self.gain + self.trim)*0.15) # GO RIGHT!
            elif distance > 0:
                motor.set_wheels_speed(left=(self.gain - self.trim)*0.1, right=(self.gain + self.trim)*0.1) # GO STRAIGHT


        if angle <= 0.0 and new_mesg != self.prev_mesg:  # STOP CONDITION
            rospy.loginfo("Destination reached")
            self.prev_mesg = new_mesg
            motor.close()

    def load_param(self):
        gain = rospy.get_param("~gain")
        trim = rospy.get_param("~trim")
        rospy.loginfo("Loaded config")

        return gain,trim





if __name__ == "__main__":

    # Initialize the node
    rospy.init_node('motor_control_node', anonymous=True, xmlrpc_port=45100, tcpros_port=45101)
    camera_node = MotorSubscriberNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down image viewer node.")