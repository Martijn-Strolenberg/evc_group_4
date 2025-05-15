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
        
        # Construct subscriber
        self.sub_cmd = rospy.Subscriber(
            "/motor_control",
            motor_cmd,
            self.read_topic,
            buff_size=2**24,
            queue_size=10
        )
        self.sample_rate = 50 # Hz

        GPIO_MOTOR_ENCODER_1=18
        GPIO_MOTOR_ENCODER_2=19
        self.driver_L = WheelEncoderDriver(GPIO_MOTOR_ENCODER_1)
        self.driver_R = WheelEncoderDriver(GPIO_MOTOR_ENCODER_2)

        # Load and initialize parameters
        self.config = self.load_param()
        self.wheel_radius = self.config["wheel_rad"]
        self.baseline = self.config["baseline"]
        self.encoder_resolution = self.config["encoder_res"]

        # Wheel encoder parameters
        self.ticks_left = 0
        self.prev_ticks_L = 0
        self.ticks_right = 0
        self.prev_ticks_R = 0
        
        ## Parameters for odom
        self.L_ticks_prev = 0
        self.R_ticks_prev = 0
        self.alpha = 2 * np.pi / self.encoder_resolution

        self.velocity = 0
        self.distance = 0
        self.angle = 0
        self.new_mesg = 0

        ## absolute position parameters
        self.x = 0
        self.y = 0
        self.theta = 0
        
        self.initialized = True
        rospy.loginfo("odem node initialized!")
        self.timer = rospy.Timer(rospy.Duration(1/self.sample_rate), self.read_encoder) # publishing at sample rate

    def read_topic(self,data):
        self.velocity = data.velocity
        self.distance = data.distance
        self.angle = data.angle
        self.new_mesg = data.new_mesg
        
    def read_encoder(self,event):
        msg = encoder() 

        # read the encoders
        self.ticks_left = self.driver_L._ticks
        self.ticks_right = self.driver_R._ticks
        d_A, d_theta = self.odometry(self.ticks_left,self.ticks_right)
        #rospy.loginfo("d_A: {delta_A},d_theta: {delta_theta} ".format(delta_A = d_A, delta_theta = d_theta))
        # Send encoder message


        self.distance -= d_A
        self.angle -= d_theta

        msg.enc_L = self.ticks_left
        msg.enc_R = self.ticks_right
        msg.d_A = d_A
        msg.d_theta = d_theta
        msg.abs_distance = self.distance
        msg.abs_angle = self.angle
        msg.new_mesg = self.new_mesg
        msg.velocity_cmd = self.velocity
        
        self.pub_odom.publish(msg)

        """
        # Difference in encoder tics from previous measurement                
        delta_ticks_left = self.ticks_left - self.prev_ticks_L
        delta_ticks_right = self.ticks_right - self.prev_ticks_R

        # update the prev ticks
        self.prev_ticks_L = self.ticks_left
        self.prev_ticks_R = self.ticks_right        
        """
        
        
    
    def odometry(self, L_ticks, R_ticks):
        msg = Odometry()
        
        # Difference in encoder tics from previous measurement                
        delta_ticks_left = L_ticks - self.L_ticks_prev
        delta_ticks_right = R_ticks - self.R_ticks_prev

        # Edge case for rotation in the same position
        if self.angle > 0:
            delta_ticks_left=delta_ticks_left*-1
        if self.angle < 0:
            delta_ticks_right=delta_ticks_right*-1

        # Update previous ticks with new tick values
        self.L_ticks_prev = L_ticks
        self.R_ticks_prev = R_ticks
        
        # Calculate rotation distance of left and right wheel in [radians]
        rotation_wheel_left = delta_ticks_left * self.alpha 
        rotation_wheel_right = delta_ticks_right * self.alpha

        # Calculate distance each wheel has travelled since last measurement in [m]
        d_left = self.wheel_radius * rotation_wheel_left
        d_right = self.wheel_radius * rotation_wheel_right

        ## Average distance travelled by the robot in [m]
        # d_A = (d_right + d_left)/2

        # This formula for d_A takes into account wheels not turning
        R = (self.baseline / 2.0) * (d_left + d_right) / (d_right - d_left)
        d_A = abs(R * d_theta)

        ## Average velocity of the robot in [m/s]
        d_v_A = d_A / (1/self.sample_rate) 

        ## How much the robot has turned (delta  )  [rads]
        d_theta = (d_right - d_left)/(self.baseline)

        self.x = self.x + self.wheel_radius * (rotation_wheel_left + rotation_wheel_right) * np.cos(self.theta)/2
        self.y = self.y + self.wheel_radius * (rotation_wheel_left + rotation_wheel_right) * np.sin(self.theta)/2
        self.theta = self.theta + self.wheel_radius * (rotation_wheel_right - rotation_wheel_left)/(self.baseline)

        ## Filling the Odometry message
        msg.header.stamp = rospy.Time.now()  # Fill the time stamp
        msg.header.frame_id = "odom"  # Fill the frame id
        msg.child_frame_id = "base_link"  # Fill the child frame id
        # fill absolute position relative to the origin 
        msg.pose.pose.position.x = self.x   # Absolute x position
        msg.pose.pose.position.y = self.y   # Absolute y position
        msg.pose.pose.position.z = 0.0      #z is always zeros as we don't drive into the air
        

        return d_A, d_theta   # return the delta average distance and delta angle


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