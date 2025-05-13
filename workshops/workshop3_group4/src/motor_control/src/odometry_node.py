#!/usr/bin/env python2

import rospy
import numpy as np

from motor_control.msg import motor_cmd, encoder
from nav_msgs.msg import Odometry # maybe for future use
from encoderDriver import *

class OdometryPublisherNode:
    def __init__(self):
        self.initialized = False
        rospy.loginfo("Initializing odometry node...")
        self.start_odometry = False
        
        # Construct publisher
        self.pub_odom = rospy.Publisher(
            "/encoder",
            encoder,
            queue_size=10
        )
        
        GPIO_MOTOR_ENCODER_1=18
        GPIO_MOTOR_ENCODER_2=19
        self.driver_L = WheelEncoderDriver(GPIO_MOTOR_ENCODER_1)
        self.driver_R = WheelEncoderDriver(GPIO_MOTOR_ENCODER_2)

        self.config = self.load_param()
        
        self.ticks_left = 0
        self.prev_ticks_L = 0
        self.ticks_right = 0
        self.prev_ticks_R = 0
        
        ## Parameters for odom
        self.L_ticks_prev = 0
        self.R_ticks_prev = 0
        self.alpha = 2 * np.pi / self.config["encoder_res"]
        
        self.initialized = True
        rospy.loginfo("odem node initialized!")
        self.timer = rospy.Timer(rospy.Duration(0.10), self.read_encoder) # publishing 10 Hz


    def read_encoder(self,event):
        msg = encoder() 

        # read the encoders
        self.ticks_left = self.driver_L._ticks
        self.ticks_right = self.driver_R._ticks
        d_A, d_theta = self.odometry(self.ticks_left,self.ticks_right)
        rospy.loginfo("d_A: {delta_A},d_theta: {delta_theta} ".format(delta_A = d_A, delta_theta = d_theta))
        # Send encoder message
        msg.enc_L = self.ticks_left
        msg.enc_R = self.ticks_right
        self.pub_odom.publish(msg)

        
        # Difference in encoder tics from previous measurement                
        delta_ticks_left = self.ticks_left - self.prev_ticks_L
        delta_ticks_right = self.ticks_right - self.prev_ticks_R

        # update the prev ticks
        self.prev_ticks_L = self.ticks_left
        self.prev_ticks_R = self.ticks_right
        


        
    
    def odometry(self, L_ticks, R_ticks):
        msg = Odometry()
        
        # Difference in encoder tics from previous measurement                
        delta_ticks_left = L_ticks - self.L_ticks_prev
        delta_ticks_right = R_ticks - self.R_ticks_prev

        # Edge case for rotation in the same position
        #if direction_left < 0:
        #    delta_ticks_left=delta_ticks_left*-1
        #if direction_right < 0:
        #    delta_ticks_right=delta_ticks_right*-1

        # Update previous ticks with new tick values
        self.L_ticks_prev = L_ticks
        self.R_ticks_prev = R_ticks
        
        # Calculate rotation distance of left and right wheel in [radians]
        rotation_wheel_left = delta_ticks_left * self.alpha 
        rotation_wheel_right = delta_ticks_right * self.alpha

        # Calculate distance each wheel has travelled since last measurement in [m]
        d_left = self.config["wheel_rad"] * rotation_wheel_left
        d_right = self.config["wheel_rad"] * rotation_wheel_right

        ## Average distance travelled by the robot in [m]
        d_A = (d_right + d_left)/2

        ## How much the robot has turned (delta  )  [rads]
        d_theta = (d_right - d_left)/(2*self.config["baseline"])

        ## Filling the Odometry message
        # fill absolute position relative to the origin 

        #self.x = self.x + self.config["wheel_rad"] * (rotation_wheel_left + rotation_wheel_right)*np.cos(self.theta)/2
        #self.y = self.y + self.config["wheel_rad"] * (rotation_wheel_left + rotation_wheel_right)*np.sin(self.theta)/2
        #self.theta = self.theta + self.config["wheel_rad"]*(rotation_wheel_right - rotation_wheel_left)/(self.config["baseline"])

        #msg.pose.pose.position.x = self.x #x
        #msg.pose.pose.position.y = self.y #y
        #msg.pose.pose.position.z = 0.0  #z is always zeros as we don't drive into the air

        return d_A,d_theta
    def load_param(self):
        config = {}
        config["gain"] = rospy.get_param("~gain")
        config["trim"] = rospy.get_param("~trim")
        config["baseline"] = rospy.get_param("~baseline")
        config["wheel_rad"] = rospy.get_param("~wheel_rad")
        config["encoder_res"] = rospy.get_param("~encoder_res")
        config["velocity"] = rospy.get_param("~velocity")
        
        rospy.loginfo("Loaded config: %s", config)
        return config        
{} 
if __name__ == "__main__":
    # Initialize the node
    rospy.init_node('odometry_node')
    odometry_node = OdometryPublisherNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down odometry node.")