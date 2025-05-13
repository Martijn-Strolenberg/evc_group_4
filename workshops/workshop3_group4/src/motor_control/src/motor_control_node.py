#!/usr/bin/env python2
import time
import hat
from hat import *
import motorDriver                       # import motor driver
from motorDriver import * # import motor driver functions
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
        #self.driver_L = WheelEncoderDriver(self.GPIO_MOTOR_ENCODER_1)
        #self.driver_R = WheelEncoderDriver(self.GPIO_MOTOR_ENCODER_2)

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

        motor = DaguWheelsDriver()

        # Get command values
        in_velocity = data.velocity
        in_distance = data.distance   # can be negative
        in_angle = data.angle         # can be negative

        angle_rad = np.deg2rad(in_angle)

        # Compute desired wheel rotation (in radians) for in-place turn
        rotate_left = angle_rad / 2.0
        rotate_right = -angle_rad / 2.0

        # Convert rotation to ticks (ticks = radians / alpha)
        left_target_ticks = abs(rotate_left / self.alpha)
        right_target_ticks = abs(rotate_right / self.alpha)

        # Record start tick positions
        left_start = self.driver_L._ticks
        right_start = self.driver_R._ticks

        # Perform rotation
        while True:
            left_progress = abs(self.driver_L._ticks - left_start)
            right_progress = abs(self.driver_R._ticks - right_start)

            curr_wheel_speed_left = 0.1 if left_progress < left_target_ticks else 0
            curr_wheel_speed_right = 0.1 if right_progress < right_target_ticks else 0

            motor.set_wheels_speed(
                left=(self.config["gain"] - self.config["trim"]) * np.sign(rotate_left) * curr_wheel_speed_left,
                right=(self.config["gain"] + self.config["trim"]) * np.sign(rotate_right) * curr_wheel_speed_right
            )

            if left_progress >= left_target_ticks and right_progress >= right_target_ticks:
                break

        # Reset encoder baselines for forward/backward driving
        left_start = self.driver_L._ticks
        right_start = self.driver_R._ticks

        # Convert linear distance to wheel ticks
        wheel_rotations = in_distance / (2 * np.pi * self.config["wheel_rad"])  # can be negative
        target_ticks = abs(wheel_rotations * self.config["encoder_res"])

        while True:
            left_progress = abs(self.driver_L._ticks - left_start)
            right_progress = abs(self.driver_R._ticks - right_start)

            curr_wheel_speed_left = 0.1 if left_progress < target_ticks else 0
            curr_wheel_speed_right = 0.1 if right_progress < target_ticks else 0

            # Distance affects direction (positive: forward, negative: backward)
            direction = np.sign(in_distance)

            motor.set_wheels_speed(
                left=(self.config["gain"] - self.config["trim"]) * direction * curr_wheel_speed_left,
                right=(self.config["gain"] + self.config["trim"]) * direction * curr_wheel_speed_right
            )

            if left_progress >= target_ticks and right_progress >= target_ticks:
                break

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