#!/usr/bin/env python2

import time
import hat
#from hat import *
import motorDriver
from motorDriver import DaguWheelsDriver
import rospy
import numpy as np
from motor_control.msg import motor_cmd, encoder
from nav_msgs.msg import Odometry
import tf
import tf.transformations as tft
from motor_control.srv import MoveStraight, MoveStraightResponse
from motor_control.srv import Rotate, RotateResponse
from motor_control.srv import Stop, StopResponse
from motor_control.srv import LeftWheelDir, RightWheelDir

class MotorSubscriberNode:
    def __init__(self):
        self.initialized = False
        rospy.loginfo("Initializing motor control node...")

        # Construct subscriber
        self.sub_cmd = rospy.Subscriber("/odom", Odometry, self.curr_position_cb, buff_size=2**24, queue_size=10)

        self.rotate_dir = 0
        self.prev_mesg = 0
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
        
        # Initialize parameters for the robot's movement goal/target
        self.goal_type = None  # 'straight' or 'rotate'
        self.goal_distance = 0 # distance to travel in meters
        self.goal_angle = 0    # angle to rotate in radians
        self.start_x = 0.0     # initial x position
        self.start_y = 0.0     # initial y position
        self.start_theta = 0.0 # initial orientation

        # Initialize state variables
        self.state = 0          # State machine variable
        self.prev_state = 0     # Previous state variable

        self.last_odom_x = 0.0
        self.last_odom_y = 0.0
        self.last_odom_theta = 0.0

        # Initialize motor command services
        rospy.Service('move_straight', MoveStraight, self.handle_move_straight)
        rospy.Service('rotate', Rotate, self.handle_rotate)
        rospy.Service('stop', Stop, self.handle_stop)

        self.motor = DaguWheelsDriver() # Initialize motor driver

        rospy.on_shutdown(self.shutdown) # Register shutdown function to clean up GPIO
        rospy.loginfo("motor control node initialized!")
        self.initialized = True

    @staticmethod
    def angle_diff(a,b):
        d = (a-b + np.pi) % (2*np.pi) - np.pi
        return d

    # ------------- service calls -------------
    def call_left_wheel_dir(self, direction):
        rospy.wait_for_service('left_wheel_dir')
        try:
            proxy = rospy.ServiceProxy('left_wheel_dir', LeftWheelDir)
            resp = proxy(direction)
            print("left wheel update success:", resp.success)
        except rospy.ServiceException as e:
            print("Service call failed:", e)

    def call_right_wheel_dir(self, direction):
        rospy.wait_for_service('right_wheel_dir')
        try:
            proxy = rospy.ServiceProxy('right_wheel_dir', RightWheelDir)
            resp = proxy(direction)
            print("right wheel update success:", resp.success)
        except rospy.ServiceException as e:
            print("Service call failed:", e)


    # ---------- service handlers ----------
    def handle_move_straight(self, req):
        if req.speed <= 0: # Check if speed is positive (error handling)
            rospy.logwarn("Speed must be > 0")
            return MoveStraightResponse(False)

        # Get the current status of the robot and set the goal parameters
        self.goal_type = 'straight'
        self.goal_distance = abs(req.distance)
        self.goal_speed = req.speed
        self.start_x = self.last_odom_x
        self.start_y = self.last_odom_y

        if req.distance > 0: # Check if distance is positive (drive forwards)
            # Move straight forward command
            rospy.loginfo("MOVING straight forwards: %.2fm at %.2fm/s", req.distance, req.speed)
            self.state = 3 # Set the state to drive straight forwards
            return MoveStraightResponse(True)

        if req.distance < 0: # Check if distance is negative (drive backwards)
            # Move straight backward command
            rospy.loginfo("MOVING straight backwards: %.2fm at %.2fm/s", req.distance, req.speed)
            self.state = 4 # Set the state to drive straight backwards
            return MoveStraightResponse(True)
        return MoveStraightResponse(False)


    def handle_rotate(self, req):
        if req.angular_speed <= 0:
            rospy.logwarn("Angular speed must be >0")
            return RotateResponse(False)

        # Get the current status of the robot and set the goal parameters
        self.goal_type = 'rotate'
        self.goal_angle = abs(req.angle)
        self.goal_speed = req.angular_speed
        self.start_theta = self.last_odom_theta

        if req.angle > 0: # Check if angle is positive (rotate clockwise)
            # Rotate clockwise command
            rospy.loginfo("ROTATING Clockwise: %.2frad at %.2frad/s", req.angle, req.angular_speed)
            self.state = 1 # Set the state to rotate clockwise
            return RotateResponse(True)
        
        if req.angle < 0: # Check if angle is negative (rotate counter-clockwise)
            # Rotate counter-clockwise command
            rospy.loginfo("ROTATING Counter-clockwise: %.2frad at %.2frad/s", req.angle, req.angular_speed)
            self.state = 2 # Set the state to rotate counter-clockwise
            return RotateResponse(True)
        return RotateResponse(False)
        

    def handle_stop(self, req):
        rospy.loginfo("STOP command received.")
        # Set the state to stop the robot
        self.state = 5
        return StopResponse(True)


    # Callback function executes when a new message is received on the /odom topic 
    def curr_position_cb(self, data):
        if not self.initialized: # Check if the node is initialized
            return               # Do nothing if not initialized

        # Extract the robot's current position and orientation from the odometry message
        self.last_odom_x = data.pose.pose.position.x
        self.last_odom_y = data.pose.pose.position.y
        yaw = tf.transformations.euler_from_quaternion([
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w])[2]
        self.last_odom_theta = yaw
        

        # === State 0: Command received ===
        if self.state == 0:
            if self.prev_state != self.state:
                rospy.loginfo("State 0: Waiting for command...")
                self.prev_state = self.state
            return # Do nothing -> Wait for a command to be received
            

        # === State 1: Rotate in place clockwise (right) ===
        if self.state == 1:
            if self.prev_state != self.state:
                rospy.loginfo("State 1: Rotate in place clockwise")
                self.prev_state = self.state
                # update the wheel directions
                self.call_left_wheel_dir(-1)  # Set left wheel direction to backward
                self.call_right_wheel_dir(1) # Set right wheel direction to forward

            if abs(self.angle_diff(self.last_odom_theta, self.start_theta)) >= self.goal_angle: # Check if the robot has rotated the desired angle
                self.state = 5 # Set the state to stop the robot
            else:
                # Set the wheel speeds for rotation right
                self.motor.set_wheels_speed(left=-(self.gain - self.trim), right=(self.gain + self.trim))
                

        # === State 2: Rotate in place counter_clockwise (left) ===
        if self.state == 2:
            if self.prev_state != self.state:
                rospy.loginfo("State 2: Rotate in place counter-clockwise")
                self.prev_state = self.state
                # update the wheel directions
                self.call_left_wheel_dir(1)  # Set left wheel direction to forward
                self.call_right_wheel_dir(-1) # Set right wheel direction to backward

            if abs(self.angle_diff(self.last_odom_theta, self.start_theta)) >= abs(self.goal_angle): # Check if the robot has rotated the desired angle
                self.state = 5 # Set the state to stop the robot
            else:
                # Set the wheel speeds for rotation left
                self.motor.set_wheels_speed(left=(self.gain - self.trim), right=-(self.gain + self.trim))

        # === State 3: Drive straight forwards ===
        if self.state == 3:
            if self.prev_state != self.state:
                rospy.loginfo("State 3: Drive straight forwards")
                self.prev_state = self.state
                # update the wheel directions
                self.call_left_wheel_dir(1)  # Set left wheel direction to forward
                self.call_right_wheel_dir(1) # Set right wheel direction to forward

            dx = self.last_odom_x - self.start_x
            dy = self.last_odom_y - self.start_y
            travelled = np.hypot(dx, dy) # Calculate the distance travelled
            if travelled >= self.goal_distance: # Check if the robot has driven the desired distance
                self.state = 5 # Set the state to stop the robot
            else:
                # Set the wheel speeds for driving straight forwards
                self.motor.set_wheels_speed(left=(self.gain - self.trim), right=(self.gain + self.trim))
            

        # === State 4: Drive straight backwards ===
        if self.state == 4:
            if self.prev_state != self.state:
                rospy.loginfo("State 4: Drive straight backwards")
                self.prev_state = self.state
                # update the wheel directions
                self.call_left_wheel_dir(-1)  # Set left wheel direction to backward
                self.call_right_wheel_dir(-1) # Set right wheel direction to backward

            dx = self.last_odom_x - self.start_x
            dy = self.last_odom_y - self.start_y
            travelled = np.hypot(dx, dy) # Calculate the distance travelled
            if travelled >= abs(self.goal_distance): # Check if the robot has driven the desired distance
                self.state = 5 # Set the state to stop the robot
            else:
                # Set the wheel speeds for driving straight backwards
                self.motor.set_wheels_speed(left=-(self.gain - self.trim), right=-(self.gain + self.trim))


        # === State 5: Finished stop robot ===
        if self.state == 5:
            if self.prev_state != self.state:
                rospy.loginfo("State 5: Stop robot")
                self.prev_state = self.state
            # Stop the robot
            self.motor.set_wheels_speed(left=0, right=0)
            self.state = 0 # Reset state to idle, ready for next command
            

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

    def shutdown(self):
        # Stop the robot and close the motor driver
        rospy.loginfo("Motor control node shutting down ...")
        self.motor.set_wheels_speed(left=0, right=0) # Stop the motors
        self.motor.close()                           # Close the motor driver                                    
        rospy.loginfo("Motor control node shut down.")
    

if __name__ == "__main__":
    # Initialize the node
    rospy.init_node('motor_control_node')
    camera_node = MotorSubscriberNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down motor_control_node.")
