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
from motor_control.msg import motor_cmd


class MotorSubscriberNode:
    def __init__(self):
        self.initialized = False
        rospy.loginfo("Initializing motor control node...")
        
        
        # Construct subscriber
        self.sub_cmd = rospy.Subscriber(
            "/motor_control",
            motor_cmd,
            self.motor_cb,
            buff_size=2**24,
            queue_size=10
        )

        self.first_image_received = False
        self.initialized = True
        rospy.loginfo("motor control node initialized!")


    def motor_cb(self, data):
        if not self.initialized:
            return
        in_vel = data.velocity
        in_pos = data.distance
        in_ang = data.angle

        motor = DaguWheelsDriver() # initialize motor drivers
        motor.set_wheels_speed(left=in_vel, right=in_vel) #
        # We need stop at the correct point in time based on encoder information

        time.sleep(5)
        motor.close() 
        

if __name__ == "__main__":
    # Initialize the node
    rospy.init_node('motor_control_node', anonymous=True, xmlrpc_port=45100, tcpros_port=45101)
    camera_node = MotorSubscriberNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down image viewer node.")

