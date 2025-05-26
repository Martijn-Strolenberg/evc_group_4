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
        distance = data.abs_distance 
        angle  = data.abs_angle
        new_mesg = data.new_mesg
        velocity_cmd = data.velocity_cmd
        action = data.move_cmd
        # To turn left or right
        #if abs(angle) > 5 * np.pi/180:
        #    if angle < 0:
        #        self.rotate_dir = 1
        #    else:
        #        self.rotate_dir = 0
        
        #     if self.rotate_dir == 1:
        #         angle = -angle

        if new_mesg != self.prev_mesg:
            motor = DaguWheelsDriver() # initialize motor drivers

            # step 1 turn the robot to the required angle
            #if angle > 0 and self.rotate_dir == 0:
            #    motor.set_wheels_speed(left=(self.gain - self.trim)*velocity_cmd, right=-(self.gain + self.trim)*velocity_cmd) # GO RIGHT!
            #elif angle > 0 and self.rotate_dir == 1:
            #    motor.set_wheels_speed(left=-(self.gain - self.trim)*velocity_cmd, right=(self.gain + self.trim)*velocity_cmd) # GO LEFT!
            #elif distance > 0:
            #    motor.set_wheels_speed(left=(self.gain - self.trim)*velocity_cmd, right=(self.gain + self.trim)*velocity_cmd) # GO STRAIGHT
            if action == 4:
                motor.set_wheels_speed(left=(self.gain - self.trim)*velocity_cmd, right=-(self.gain + self.trim)*velocity_cmd) # GO RIGHT!
            elif action == 3:
                motor.set_wheels_speed(left=-(self.gain - self.trim)*velocity_cmd, right=(self.gain + self.trim)*velocity_cmd) # GO LEFT!
            elif action == 1:
                motor.set_wheels_speed(left=(self.gain - self.trim)*velocity_cmd, right=(self.gain + self.trim)*velocity_cmd) # GO STRAIGHT    
            elif action == 2:
                motor.set_wheels_speed(left=-(self.gain - self.trim)*velocity_cmd, right=-(self.gain + self.trim)*velocity_cmd) # GO BACK     
        #if angle <= 5*np.pi/180 and distance < 0.01 and new_mesg != self.prev_mesg:  # STOP CONDITION
        #    rospy.loginfo("Destination reached")
        #    self.prev_mesg = new_mesg
        #    motor.close()
        rospy.loginfo("Action:{act}".format(act = action))
        if action == 0 and new_mesg != self.prev_mesg:  # STOP CONDITION
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



        # if not self.initialized:
        #     return
        
        # motor = DaguWheelsDriver() # initialize motor drivers
        
        # # Get the values from the topic
        # in_velocity = data.velocity
        # in_distance = data.distance
        # in_angle = data.angle

        # # convert degrees to rads
        # angle_rad = np.deg2rad(in_angle)
        
        # # sin(theta) = x / dist .'. 
        # # x = sin(theta) * dist
        # new_x = np.sin(angle_rad) * in_distance
        
        # # cos(theta) = y / dist .'. 
        # # y = cos(theta) * dist
        # new_y = np.cos(angle_rad) * in_distance
        
        # # Rotate the robot with in_angle degrees
        # if in_angle > 0:
        #     rotate_left = angle_rad / 2
        #     rotate_right = -angle_rad / 2
        # else:
        #     rotate_left = -angle_rad / 2
        #     rotate_right = angle_rad / 2
        
        # rotation_left_done = 0
        # rotation_right_done = 0

        # old_right_motor_encoder_ticks = self.driver_R._ticks
        # old_left_motor_encoder_ticks = self.driver_L._ticks
        
        # # continue untill both rotations have finished
        # while (rotation_left_done == 0) or (rotation_right_done == 0):
        #     right_motor_encoder_ticks = old_right_motor_encoder_ticks - self.driver_R._ticks
        #     left_motor_encoder_ticks = old_left_motor_encoder_ticks - self.driver_L._ticks

        #     if(rotation_left_done == 0) and (self.alpha * rotate_left > left_motor_encoder_ticks):
        #         rotation_left_done = 0
        #         curr_wheel_speed_left = 0.1
        #     else:
        #         rotation_left_done = 1
        #         curr_wheel_speed_left = 0

        #     if(rotation_right_done == 0) and (self.alpha * rotate_right > right_motor_encoder_ticks):
        #         rotation_right_done = 0
        #         curr_wheel_speed_right = 0.1
        #     else:
        #         rotation_right_done = 1
        #         curr_wheel_speed_right = 0
            
        #     motor.set_wheels_speed(left=(self.config["gain"] - self.config["trim"])*curr_wheel_speed_left, 
        #                        right=(self.config["gain"] + self.config["trim"])*curr_wheel_speed_right)

        # old_right_motor_encoder_ticks = self.driver_R._ticks
        # old_left_motor_encoder_ticks = self.driver_L._ticks

        # # Drive a certain distance
        # right_motor_encoder_ticks = in_distance * self.alpha / 2 * np.pi * self.config["wheel_rad"]
        # left_motor_encoder_ticks = in_distance * self.alpha / 2 * np.pi * self.config["wheel_rad"]
        
        # while (rotation_left_done == 0) or (rotation_right_done == 0):
        #     right_motor_encoder_ticks = old_right_motor_encoder_ticks - self.driver_R._ticks
        #     left_motor_encoder_ticks = old_left_motor_encoder_ticks - self.driver_L._ticks

        #     if(rotation_left_done == 0) and (self.alpha * rotate_left > left_motor_encoder_ticks):
        #         rotation_left_done = 0
        #         curr_wheel_speed_left = 0.1
        #     else:
        #         rotation_left_done = 1
        #         curr_wheel_speed_left = 0

        #     if(rotation_right_done == 0) and (self.alpha * rotate_right > right_motor_encoder_ticks):
        #         rotation_right_done = 0
        #         curr_wheel_speed_right = 0.1
        #     else:
        #         rotation_right_done = 1
        #         curr_wheel_speed_right = 0
            
        #     motor.set_wheels_speed(left=(self.config["gain"] - self.config["trim"])*curr_wheel_speed_left, 
        #                        right=(self.config["gain"] + self.config["trim"])*curr_wheel_speed_right)
        # motor.close()