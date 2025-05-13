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
from nav_msgs.msg import Odometry


class MotorSubscriberNode:
    def __init__(self):
        self.initialized = False
        rospy.loginfo("Initializing motor control node...")
        self.start_odometry = False
        
        # Construct subscriber
        self.sub_cmd = rospy.Subscriber(
            "/motor_control",
            motor_cmd,
            self.motor_cb,
            buff_size=2**24,
            queue_size=10
        )
        ## Get intrinsic Robot params
        self.config = self.load_param()
        self.gain = self.config["gain"]
        self.trim = self.config["trim"]
        self.Radius = self.config["wheel_rad"]
        self.Radius = self.config["wheel_rad"]
        
        self.first_image_received = False
        self.initialized = True
        
        # Init ticks to starting encoder values: 
        self.L_ticks = 0 
        self.R_ticks = 0

        self.x_pos_origin = self.x_pos_origin + x_pos_rel
        self.y_pos_origin = self.y_pos_origin + y_pos_rels
        self.theta_pos_origin = self.theta_pos_origin + d_theta

        rospy.loginfo("motor control node initialized!")


    def motor_cb(self, data):
        if not self.initialized:
            return
        in_vel = -data.velocity
        in_pos = data.distance
        in_ang = data.angle
        
        motor = DaguWheelsDriver() # initialize motor drivers
        motor.set_wheels_speed(left=(self.gain - self.trim)*0, 
                               right=(self.gain + self.trim)*in_vel) #
        # We need stop at the correct point in time based on encoder information

        time.sleep(8)
        motor.close() 

    ## Measures the odometry and returns values since last measurement.
    # @param direction 1 for forward 0 for backwards
    def odometry(self,L_ticks,R_ticks,direction_left,direction_right):
        msg = Odometry()
        
        # Radians per tick
        alpha = 2 * np.pi / self.config["encoder_res"]
        
        # Difference in encoder tics from previous measurement                
        delta_ticks_left = L_ticks - self.L_ticks_prev
        delta_ticks_right = R_ticks - self.R_ticks_prev

        # Edge case for ro
 gnita
        if direction_left < 0:
            delta_ticks_left=delta_ticks_left*-1
        if direction_right < 0:
            delta_ticks_right=delta_ticks_right*-1

        # upd
        self.L_ticks_prev = L_ticks
        self.R_ticks_prev = R_ticks
        
        rotation_wheel_left = delta_ticks_left * alpha 
        rotation_wheel_right = delta_ticks_right * alpha

        d_left = self.Radius * rotation_wheel_left
        d_right = self.Radius * rotation_wheel_right
        
        #d_A = (d_left+d_right)/2
        #Delta_Theta = (d_right-d_left)/2*se

        ## Average distance travelled by the robot
        d_A = (d_right + d_left)/2
        ## How much the robot has turned (delta theta)
        d_theta = (d_right - d_left)/(2*self.config["baseline"])
        x_pos_rel=d_A*np.cos(d_theta)
        y_pos_rel=d_A*np.sin(d_theta)

        self.x_pos_origin = self.x_pos_origin + x_pos_rel
        self.y_pos_origin = self.y_pos_origin + y_pos_rel
        self.theta_pos_origin = self.theta_pos_origin + d_theta
        ## Filling the Odometry message
        # fill absolute position relative to thhe origin 
        msg.pose.pose.position.x = #x
        msg.pose.pose.position.y = #y
        msg.pose.pose.position.y = 0.0 #z is always zeros as we don't drive into the air



        return L_ticks, R_ticks, position
        
    def del_phi(self,L_ticks,R_ticks,direction_left,direction_right)

    
    def load_param(self):
        config = {}
        config["gain"] = rospy.get_param("~gain")
        config["trim"] = rospy.get_param("~trim")
        config["baseline"] = rospy.get_param("~baseline")
        config["wheel_rad"] = rospy.get_param("~wheel_rad")
        config["encoder_res"] = rospy.get_param("~encoder_res")
        
        rospy.loginfo("Loaded config: %s", config)
        return config
        

if __name__ == "__main__":
    # Initialize the node
    rospy.init_node('motor_control_node', anonymous=True, xmlrpc_port=45102, tcpros_port=45103)
    camera_node = MotorSubscriberNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down image viewer node.")

