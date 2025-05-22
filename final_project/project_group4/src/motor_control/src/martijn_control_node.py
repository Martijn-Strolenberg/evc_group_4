#!/usr/bin/env python2

import time
#import hat
from hat import *
import motorDriver
from motorDriver import DaguWheelsDriver
import rospy
import numpy as np
from motor_control.msg import motor_cmd, encoder

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
        self.prev_mesg = 0

        rospy.loginfo("motor control node initialized!")

        self.state = 0

        self.config = self.load_param()
        self.wheel_radius = self.config["wheel_rad"]
        self.baseline = self.config["baseline"]
        self.encoder_resolution = self.config["encoder_res"]
        self.gain = self.config["gain"]
        self.trim = self.config["trim"]

        self.target_enc_L = 0
        self.target_enc_R = 0
        self.alpha = 2 * np.pi / self.encoder_resolution
        self.curr_msg = -1
        self.initialized = True

    def motor_cb(self, data):
        if not self.initialized:
            return
        if data.new_mesg > self.curr_msg:
            self.curr_msg = data.new_mesg

        if not data.blocking and self.curr_msg != data.new_mesg:
            return
        
        motor = DaguWheelsDriver()

        # Extract updated encoder ticks from the incoming message
        current_left_ticks = data.enc_L
        current_right_ticks = data.enc_R

        # === State 0: Command received ===
        if self.state == 0:
            if (self.curr_msg == data.new_mesg):
                # Do nothing... no new motor command received
                return
            else:
                self.abs_distance_old = data.abs_distance
                self.abs_angle_old = data.abs_angle
                rospy.loginfo("State = 0 Running Initialization")

                rospy.loginfo("curr_msg: {:.2f} new_mesg: {:.2f}".format(self.curr_msg, data.new_mesg))

                # Cache input command
                self.in_distance = data.abs_distance
                if not data.blocking:
                    self.in_distance = 100000000

                self.in_angle_rad = data.abs_angle

                # Cache initial encoder readings
                self.left_start_ticks = current_left_ticks
                self.right_start_ticks = current_right_ticks

                # Compute target ticks for rotation
                self.rotate_left = self.in_angle_rad
                self.rotate_right = -self.in_angle_rad

                self.left_target_rot_ticks = abs(self.rotate_left / self.alpha)*2    # calculate theoretical encoder values
                self.right_target_rot_ticks = abs(self.rotate_right / self.alpha)*2

                rospy.loginfo("in_distance: {:.2f} \tin_angle_rad: {:.2f}".format(self.in_distance, self.in_angle_rad))

                self.rot_speed = data.velocity_cmd
                self.drive_speed = data.velocity_cmd

                self.state = 1  # Go to rotation phase
                return

        # === State 1: Rotate in place ===
        if self.state == 1:
            rospy.loginfo("State = 1 Rotating in Place")
            
            left_rot_progress = abs(current_left_ticks - self.left_start_ticks)
            right_rot_progress = abs(current_right_ticks - self.right_start_ticks)

            left_done = left_rot_progress >= self.left_target_rot_ticks
            right_done = right_rot_progress >= self.right_target_rot_ticks

            # Speed zero if wheel is done
            curr_wheel_speed_left = self.rot_speed if not left_done else 0
            curr_wheel_speed_right = self.rot_speed if not right_done else 0

            # Set directions explicitly based on angle sign
            if self.in_angle_rad > 0:
                rospy.loginfo("\tRotating LEFT")
                # Turn left: left forward, right backward
                left_speed = curr_wheel_speed_left
                right_speed = -curr_wheel_speed_right
            else:
                rospy.loginfo("\tRotating RIGHT")
                # Turn right: left backward, right forward
                left_speed = -curr_wheel_speed_left
                right_speed = curr_wheel_speed_right

            motor.set_wheels_speed(
                left=(self.gain - self.trim) * left_speed,
                right=(self.gain + self.trim) * right_speed
            )

            if left_done and right_done:
                rospy.loginfo("\tDone rotating")
                self.state = 2  # Prepare for driving straght
                self.left_start_ticks = current_left_ticks
                self.right_start_ticks = current_right_ticks
            return

        # === State 2: Drive straight ===
        if self.state == 2:
            rospy.loginfo("State = 2 Driving Straight Distance")
            wheel_rotations = self.in_distance / (2 * np.pi * self.wheel_radius)
            target_drive_ticks = abs(wheel_rotations * self.encoder_resolution)

            left_drive_progress = abs(current_left_ticks - self.left_start_ticks)
            right_drive_progress = abs(current_right_ticks - self.right_start_ticks)

            left_done = left_drive_progress >= target_drive_ticks
            right_done = right_drive_progress >= target_drive_ticks

            direction = np.sign(self.in_distance)

            curr_wheel_speed_left = self.drive_speed if not left_done else 0
            curr_wheel_speed_right = self.drive_speed if not right_done else 0

            motor.set_wheels_speed(
                left=(self.gain - self.trim) * direction * curr_wheel_speed_left,
                right=(self.gain + self.trim) * direction * curr_wheel_speed_right
            )

            if left_done and right_done:
                motor.set_wheels_speed(left=0, right=0)
                motor.close()
                self.state = 3  # Finished
            return

        # === State 3: Finished ===
        if self.state == 3:
            rospy.loginfo("State = 3 Motion is Complete.")
            rospy.loginfo("Motion complete.")
            self.state = 0  # Ready for next command
            self.curr_msg = data.new_mesg
            return

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
    rospy.init_node('motor_control_node')
    camera_node = MotorSubscriberNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down motor_control_node.")
