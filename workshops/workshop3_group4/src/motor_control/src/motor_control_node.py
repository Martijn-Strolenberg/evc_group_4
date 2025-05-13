#!/usr/bin/env python2
import time
import hat
from hat import *
import motorDriver                       # import motor driver
from motorDriver import DaguWheelsDriver # import motor driver functions
from encoderDriver import *              # import encoder drivers
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

        # Encoder pins 
        self.GPIO_MOTOR_ENCODER_1=18
        self.GPIO_MOTOR_ENCODER_2=19
        self.gpio_pin = 19
        self.driver_L = WheelEncoderDriver(self.GPIO_MOTOR_ENCODER_1)
        self.driver_R = WheelEncoderDriver(self.GPIO_MOTOR_ENCODER_2)

        ## Get intrinsic Robot params
        self.config = self.load_param()
        self.y = 0
        self.x = 0
        self.theta = 0
        
        # Radians per tick
        self.alpha = 2 * np.pi / self.config["encoder_res"]
        
        self.first_image_received = False
        self.initialized = True
        
        # Init prev ticks
        self.L_ticks_prev = 0
        self.R_ticks_prev = 0
        rospy.loginfo("motor control node initialized!")


    def motor_cb(self, data):
        if not self.initialized:
            return
        
        motor = DaguWheelsDriver() # initialize motor drivers
        
        # Get the values from the topic
        in_velocity = data.velocity
        in_distance = data.distance
        in_angle = data.angle

        # convert degrees to rads
        angle_rad = np.deg2rad(in_angle)
        
        # sin(theta) = x / dist .'. 
        # x = sin(theta) * dist
        new_x = np.sin(angle_rad) * in_distance
        
        # cos(theta) = y / dist .'. 
        # y = cos(theta) * dist
        new_y = np.cos(angle_rad) * in_distance
        
        # Rotate the robot with in_angle degrees
        if in_angle > 0:
            rotate_left = angle_rad / 2
            rotate_right = -angle_rad / 2
        else:
            rotate_left = -angle_rad / 2
            rotate_right = angle_rad / 2
        
        rotation_left_done = 0
        rotation_right_done = 0

        old_right_motor_encoder_ticks = self.driver_R._ticks
        old_left_motor_encoder_ticks = self.driver_L._ticks
        
        # continue untill both rotations have finished
        while (rotation_left_done == 0) or (rotation_right_done == 0):
            right_motor_encoder_ticks = old_right_motor_encoder_ticks - self.driver_R._ticks
            left_motor_encoder_ticks = old_left_motor_encoder_ticks - self.driver_L._ticks

            if(rotation_left_done == 0) and (self.alpha * rotate_left > left_motor_encoder_ticks):
                rotation_left_done = 0
                curr_wheel_speed_left = 0.1
            else:
                rotation_left_done = 1
                curr_wheel_speed_left = 0

            if(rotation_right_done == 0) and (self.alpha * rotate_right > right_motor_encoder_ticks):
                rotation_right_done = 0
                curr_wheel_speed_right = 0.1
            else:
                rotation_right_done = 1
                curr_wheel_speed_right = 0
            
            motor.set_wheels_speed(left=(self.config["gain"] - self.config["trim"])*curr_wheel_speed_left, 
                               right=(self.config["gain"] + self.config["trim"])*curr_wheel_speed_right)
                

            
        # motor.set_wheels_speed(left=(self.config["gain"] - self.config["trim"])*rotate_left, 
        #                        right=(self.config["gain"] + self.config["trim"])*rotate_right) #
        # # We need stop at the correct point in time based on encoder information

        # if in_distance <= np.sqrt(self.x ** 2 + self.y ** 2):
            
        # #time.sleep(8)
            motor.close() 

    ## Measures the odometry and returns values since last measurement.
    # @param direction 1 for forward 0 for backwards
    ## Measures the odometry and returns values since last measurement.
    # @param direction 1 for forward 0 for backwards
    def odometry(self,L_ticks,R_ticks,direction_left,direction_right):
        msg = Odometry()
        
        # Difference in encoder tics from previous measurement                
        delta_ticks_left = L_ticks - self.L_ticks_prev
        delta_ticks_right = R_ticks - self.R_ticks_prev

        # Edge case for rotation in the same position
        if direction_left < 0:
            delta_ticks_left=delta_ticks_left*-1
        if direction_right < 0:
            delta_ticks_right=delta_ticks_right*-1

        # Update previous ticks with new tick values
        self.L_ticks_prev = L_ticks
        self.R_ticks_prev = R_ticks
        
        # Calculate rotation distance of left and right wheel in radiants
        rotation_wheel_left = delta_ticks_left * self.alpha 
        rotation_wheel_right = delta_ticks_right * self.alpha

        # Calculate distance each wheel has travelled since last measurement
        d_left = self.config["wheel_rad"] * rotation_wheel_left
        d_right = self.config["wheel_rad"] * rotation_wheel_right

        ## Average distance travelled by the robot
        d_A = (d_right + d_left)/2

        ## How much the robot has turned (delta  )
        d_theta = (d_right - d_left)/(2*self.config["baseline"])

        ## Filling the Odometry message
        # fill absolute position relative to the origin 

        self.x = self.x + self.config["wheel_rad"] * (rotation_wheel_left + rotation_wheel_right)*np.cos(self.theta)/2
        self.y = self.y + self.config["wheel_rad"] * (rotation_wheel_left + rotation_wheel_right)*np.sin(self.theta)/2
        self.theta = self.theta + self.config["wheel_rad"]*(rotation_wheel_right - rotation_wheel_left)/(self.config["baseline"])

        msg.pose.pose.position.x = self.x #x
        msg.pose.pose.position.y = self.y #y
        msg.pose.pose.position.z = 0.0  #z is always zeros as we don't drive into the air

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
        

if __name__ == "__main__":
    # Initialize the node
    rospy.init_node('motor_control_node', anonymous=True, xmlrpc_port=45102, tcpros_port=45103)
    camera_node = MotorSubscriberNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down image viewer node.")