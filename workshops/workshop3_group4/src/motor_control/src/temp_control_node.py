#!/usr/bin/env python2

import time
import hat
from hat import *
import motorDriver
from motorDriver import DaguWheelsDriver
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
        self.rotate_dir = 0
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
        velocity_cmd = data.velocity_cmd
        # To turn left or right
        if angle < 0:
            self.rotate_dir = 1
        else:
            self.rotate_dir = 0
        
        if self.rotate_dir == 1:
            angle = -angle

        if new_mesg != self.prev_mesg:
            motor = DaguWheelsDriver() # initialize motor drivers

            # step 1 turn the robot to the required angle
            if angle > 0 and self.rotate_dir == 0:
                #motor.set_wheels_speed(left=-(self.gain - self.trim)*0.1, right=(self.gain + self.trim)*0.1) # GO LEFT!
                motor.set_wheels_speed(left=(self.gain - self.trim)*0.15, right=-(self.gain + self.trim)*0.15) # GO RIGHT!
            elif angle > 0 and self.rotate_dir == 1:
                motor.set_wheels_speed(left=-(self.gain - self.trim)*0.1, right=(self.gain + self.trim)*0.1) # GO LEFT!
            elif distance > 0:
                motor.set_wheels_speed(left=(self.gain - self.trim)*0.1, right=(self.gain + self.trim)*0.1) # GO STRAIGHT
        
        if angle <= 5*np.pi/180 and new_mesg != self.prev_mesg:  # STOP CONDITION
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