#!/usr/bin/env python2

from motor_control.drivers.motorDriver import DaguWheelsDriver
import rospy
import numpy as np
from nav_msgs.msg import Odometry
import tf
import tf.transformations as tft
#from motor_control import MotorServices
from motor_control.srv import MoveStraight, MoveStraightResponse
from motor_control.srv import Rotate, RotateResponse
from motor_control.srv import Stop, StopResponse
from motor_control.srv import LeftWheelDir, RightWheelDir
from motor_control.srv import ConstRotate, ConstRotateResponse
from motor_control.srv import ConstStraight, ConstStraightResponse
from motor_control.srv import DriveLeftwheel, DriveLeftwheelResponse
from motor_control.srv import DriveRightwheel, DriveRightwheelResponse

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
        self.goal_speed = 0.0   # speed in m/s or rad/s
        self.goal_left_speed = 0.0  # speed for left wheel in m/s
        self.goal_right_speed = 0.0 # speed for right wheel in m/s
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

        # PID state variables
        self.Kp = self.config["Kp"]  # Proportional gain
        self.Ki = self.config["Ki"]  # Integral gain
        self.Kd = self.config["Kd"]  # Derivative gain
        self.prev_theta_error = 0.0  # Previous heading error
        self.integral_error = 0.0    # Integral of error
        self.last_pid_time = rospy.Time.now()  # Last time PID was computed

        # Motor limit parameters
        self.max_velocity = self.config["max_velocity"]  # Maximum velocity for the robot
        self.acceleration = self.config["acceleration"]  # Acceleration limit
        self.deceleration = self.config["deceleration"]  # Deceleration limit
        self.cmd_left = 0.0                              # Current command for left wheel
        self.cmd_right = 0.0                             # Current command for right wheel
        self.last_cmd_time = rospy.Time.now()            # Last time a command was sent

        # Initialize motor command services API
        #services = MotorServices()
        rospy.Service('move_straight', MoveStraight, self.handle_move_straight)
        rospy.Service('rotate', Rotate, self.handle_rotate)
        rospy.Service('stop', Stop, self.handle_stop)
        rospy.Service('const_rotate', ConstRotate, self.handle_const_rotate)
        rospy.Service('const_straight', ConstStraight, self.handle_const_straight)
        rospy.Service('left_wheel_vel', DriveLeftwheel, self.handle_drive_leftwheel)
        rospy.Service('right_wheel_vel', DriveRightwheel, self.handle_drive_rightwheel)

        self.motor = DaguWheelsDriver() # Initialize motor driver

        rospy.on_shutdown(self.shutdown) # Register shutdown function to clean up GPIO
        rospy.loginfo("Motor control node initialized!")
        self.initialized = True

    @staticmethod # Function to calculate the difference between two angles
    def angle_diff(a, b):
        difference = (a-b + np.pi) % (2*np.pi) - np.pi
        return difference

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
        
    
    def handle_const_rotate(self, req):
        if req.angular_speed <= 0:
            rospy.logwarn("Speed must be >0")
            return ConstRotateResponse(False)
        
        self.goal_type = 'const_rotate'
        self.goal_speed = req.angular_speed

        if req.direction > 0: # Check if direction is positive (rotate clockwise) (right?)
            # Rotate clockwise command
            rospy.loginfo("CONSTANT ROTATING Clockwise: at %.2frad/s", req.angular_speed)
            self.state = 9 # Set the state to constant rotate clockwise
            return ConstRotateResponse(True)
        
        if req.direction < 0: # Check if direction is negative (rotate counter-clockwise) (left?)
            # Rotate counter-clockwise command
            rospy.loginfo("CONSTANT ROTATING Counter-clockwise:  at %.2frad/s", req.angular_speed)
            self.state = 8 # Set the state to constant rotate counter-clockwise
            return ConstRotateResponse(True)
        return ConstRotateResponse(False)

    def handle_const_straight(self, req):
        if req.speed <= 0:
            rospy.logwarn("Speed must be >0")
            return ConstRotateResponse(False)
        # Get the current status of the robot and set the goal parameters
        
        self.goal_type = 'const_straight'
        self.goal_speed = req.speed

        if req.direction > 0: # Check if distance is positive (drive forwards)
            # Move straight forward command
            rospy.loginfo("CONSTANT MOVING straight forwards %.2fm/s", req.speed)
            self.state = 6 # Set the state to drive straight forwards
            return ConstStraightResponse(True)

        if req.direction < 0: # Check if distance is negative (drive backwards)
            # Move straight backward command
            rospy.loginfo("CONSTANT MOVING straight backwards %.2fm/s", req.speed)
            self.state = 7 # Set the state to drive straight backwards
            return ConstStraightResponse(True)
        return ConstStraightResponse(False)
    
    def handle_drive_leftwheel(self, req):
        if req.speed <= 0:
            rospy.logwarn("Speed must be >0")
            return DriveLeftwheelResponse(False)
        
        self.goal_left_speed = req.speed # Set the desired speed of the motors

        if req.direction > 0: # Check if distance is positive (drive forwards)
            # Move straight forward command
            rospy.loginfo("Left wheel forwards %.2fm/s", req.speed)
            self.state = 10 # Set the state to drive left wheel forwards
            return DriveLeftwheelResponse(True)
        
        if req.direction < 0: # Check if distance is negative (drive backwards)
            # Move straight backward command
            rospy.loginfo("Left wheel backwards %.2fm/s", req.speed)
            self.state = 11 # Set the state to drive left wheel backwards
            return DriveLeftwheelResponse(True)
        return DriveLeftwheelResponse(False)

    def handle_drive_rightwheel(self, req):
        if req.speed <= 0:
            rospy.logwarn("Speed must be >0")
            return DriveRightwheelResponse(False)
        
        self.goal_right_speed = req.speed # Set the desired speed of the motors
        
        if req.direction > 0: # Check if distance is positive (drive forwards)
            # Move straight forward command
            rospy.loginfo("Right wheel forwards %.2fm/s", req.speed)
            self.state = 12 # Set the state to drive right wheel forwards
            return DriveRightwheelResponse(True)
        
        if req.direction < 0: # Check if distance is negative (drive backwards)
            # Move straight backward command
            rospy.loginfo("Right wheel backwards %.2fm/s", req.speed)
            self.state = 13 # Set the state to drive right wheel backwards
            return DriveRightwheelResponse(True)
        return DriveRightwheelResponse(False)
    

##### STATE MACHINE
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
                #self.motor.set_wheels_speed(left=-(self.gain - self.trim)*self.goal_speed, right=(self.gain + self.trim)*self.goal_speed)
                target_left = -self.goal_speed
                target_right = self.goal_speed
                left_cmd, right_cmd = self.acceleration_func(target_left, target_right) # Apply acceleration limits
                self.motor.set_wheels_speed(left=(self.gain - self.trim)*left_cmd, right=(self.gain - self.trim)*right_cmd)
                

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
                #self.motor.set_wheels_speed(left=(self.gain - self.trim)*self.goal_speed, right=-(self.gain + self.trim)*self.goal_speed)
                target_left = self.goal_speed
                target_right = -self.goal_speed
                left_cmd, right_cmd = self.acceleration_func(target_left, target_right) # Apply acceleration limits
                self.motor.set_wheels_speed(left=(self.gain - self.trim)*left_cmd, right=(self.gain - self.trim)*right_cmd)

        # === State 3: Drive straight forwards ===
        if self.state == 3:
            if self.prev_state != self.state:
                rospy.loginfo("State 3: Drive straight forwards")
                self.prev_state = self.state
                # update the wheel directions
                self.call_left_wheel_dir(1)  # Set left wheel direction to forward
                self.call_right_wheel_dir(1) # Set right wheel direction to forward
                self.start_theta = self.last_odom_theta # Store the initial orientation
                self.cmd_left = 0
                self.cmd_right = 0

            dx = self.last_odom_x - self.start_x
            dy = self.last_odom_y - self.start_y
            travelled = np.hypot(dx, dy) # Calculate the distance travelled
            if travelled >= self.goal_distance: # Check if the robot has driven the desired distance
                self.state = 5 # Set the state to stop the robot
            else:
                # Set the wheel speeds for driving straight forwards
                # self.motor.set_wheels_speed(left=(self.gain - self.trim)*self.goal_speed, right=(self.gain + self.trim)*self.goal_speed)

                # -------- PID heading control -----------
                v_nom =  self.goal_speed                 # positive forward
                v, omega = self.pid_heading_control(v_nom, self.start_theta, self.last_odom_theta)
                left_cmd, right_cmd = self.v_omega_to_motor_cmd(v, omega)
                left_cmd, right_cmd = self.acceleration_func(left_cmd, right_cmd) # Apply acceleration limits
                self.motor.set_wheels_speed(left_cmd, right_cmd)
                
        # === State 4: Drive straight backwards ===
        if self.state == 4:
            if self.prev_state != self.state:
                rospy.loginfo("State 4: Drive straight backwards")
                self.prev_state = self.state
                # update the wheel directions
                self.call_left_wheel_dir(-1)  # Set left wheel direction to backward
                self.call_right_wheel_dir(-1) # Set right wheel direction to backward
                self.start_theta = self.last_odom_theta # Store the initial orientation
                self.cmd_left = 0
                self.cmd_right = 0

            dx = self.last_odom_x - self.start_x
            dy = self.last_odom_y - self.start_y
            travelled = np.hypot(dx, dy) # Calculate the distance travelled
            if travelled >= abs(self.goal_distance): # Check if the robot has driven the desired distance
                self.state = 5 # Set the state to stop the robot
            else:
                # Set the wheel speeds for driving straight backwards
                # self.motor.set_wheels_speed(left=-(self.gain - self.trim)*self.goal_speed, right=-(self.gain + self.trim)*self.goal_speed)
                # -------- PID heading control -----------
                v_nom =  -self.goal_speed                 # positive forward
                v, omega = self.pid_heading_control(v_nom, self.start_theta, self.last_odom_theta)
                left_cmd, right_cmd = self.v_omega_to_motor_cmd(v, omega)
                left_cmd, right_cmd = self.acceleration_func(left_cmd, right_cmd) # Apply acceleration limits
                self.motor.set_wheels_speed(left_cmd, right_cmd)

        # === State 5: Finished stop robot ===
        if self.state == 5:
            if self.prev_state != self.state:
                rospy.loginfo("State 5: Stop robot")
                self.prev_state = self.state
            # Stop the robot
            #self.motor.set_wheels_speed(left=0, right=0) # Stop the robot without any deceleration
            cmd_left, cmd_right = self.acceleration_func(0, 0) # Apply acceleration limits to stop
            self.motor.set_wheels_speed(left=cmd_left, right=cmd_right) # Stop the robot
            self.state = 0 # Reset state to idle, ready for next command

        # === State 6: constant drive robot forwards ===
        if self.state == 6:
            if self.prev_state != self.state:
                rospy.loginfo("State 6: Drive constant Forwards")
                self.prev_state = self.state
                # update the wheel directions
                self.call_left_wheel_dir(1)  # Set left wheel direction to forward
                self.call_right_wheel_dir(1) # Set right wheel direction to forward
                self.start_theta = self.last_odom_theta # Store the initial orientation
                self.cmd_left = 0
                self.cmd_right = 0

            # self.motor.set_wheels_speed(left=(self.gain - self.trim)*self.goal_speed, right=(self.gain + self.trim)*self.goal_speed) 
            # -------- PID heading control -----------
            v_nom =  self.goal_speed                 # positive forward
            v, omega = self.pid_heading_control(v_nom, self.start_theta, self.last_odom_theta)
            left_cmd, right_cmd = self.v_omega_to_motor_cmd(v, omega)
            left_cmd, right_cmd = self.acceleration_func(left_cmd, right_cmd) # Apply acceleration limits
            self.motor.set_wheels_speed(left_cmd, right_cmd)

        # === State 7: constant drive robot backwards ===      
        if self.state == 7:
            if self.prev_state != self.state:
                rospy.loginfo("State 7: Drive constant backwards")
                self.prev_state = self.state
                # update the wheel directions
                self.call_left_wheel_dir(-1)  # Set left wheel direction to backward
                self.call_right_wheel_dir(-1) # Set right wheel direction to backward
                self.start_theta = self.last_odom_theta # Store the initial orientation
                self.cmd_left = 0
                self.cmd_right = 0

            # self.motor.set_wheels_speed(left=-(self.gain - self.trim)*self.goal_speed, right=-(self.gain + self.trim)*self.goal_speed)  
            # -------- PID heading control -----------
            v_nom =  -self.goal_speed                 # positive forward
            v, omega = self.pid_heading_control(v_nom, self.start_theta, self.last_odom_theta)
            left_cmd, right_cmd = self.v_omega_to_motor_cmd(v, omega)
            left_cmd, right_cmd = self.acceleration_func(left_cmd, right_cmd) # Apply acceleration limits
            self.motor.set_wheels_speed(left_cmd, right_cmd)

        # === State 8: constant turn robot left ===
        if self.state == 8:
            if self.prev_state != self.state:
                rospy.loginfo("State 8: Rotate constant left") # (counter clockwise)
                self.prev_state = self.state
                # update the wheel directions
                self.call_left_wheel_dir(1)  # Set left wheel direction to forward
                self.call_right_wheel_dir(-1) # Set right wheel direction to backward

                self.start_theta = self.last_odom_theta # Store the initial orientation

            self.motor.set_wheels_speed(left=(self.gain - self.trim)*self.goal_speed, 
                                        right=-(self.gain + self.trim)*self.goal_speed) 
              
        # === State 9: constant turn robot right ===
        if self.state == 9:
            if self.prev_state != self.state:
                rospy.loginfo("State 9: Rotate constant right") # (clockwise)
                self.prev_state = self.state
                # update the wheel directions
                self.call_left_wheel_dir(-1)  # Set left wheel direction to backward
                self.call_right_wheel_dir(1) # Set right wheel direction to forward
                self.start_theta = self.last_odom_theta # Store the initial orientation

            self.motor.set_wheels_speed(left=-(self.gain - self.trim)*self.goal_speed, 
                                        right=(self.gain + self.trim)*self.goal_speed)  
        
        # === State 10: only command left wheel to move forwards  ===
        if self.state == 10:
            if self.prev_state != self.state:
                rospy.loginfo("State 10: command left wheel forwards")
                self.prev_state = self.state
                # update left wheel direction
                self.call_left_wheel_dir(1)  # Set left wheel direction to forwards
            #self.motor.set_wheels_speed(left=(self.gain - self.trim)*self.goal_left_speed, right=(self.gain + self.trim)*self.goal_right_speed)
            desired_left = (self.gain - self.trim)*self.goal_left_speed
            desired_right = (self.gain + self.trim)*self.goal_right_speed
            left_cmd, right_cmd = self.acceleration_func(desired_left, desired_right) # Apply acceleration limits
            self.motor.set_wheels_speed(left=left_cmd, right=right_cmd) # Set the wheel speeds

        # === State 11: only command left wheel to move backwars  ===
        if self.state == 11:
            if self.prev_state != self.state:
                rospy.loginfo("State 11: command left wheel backwards")
                self.prev_state = self.state
                # update left wheel direction
                self.call_left_wheel_dir(-1)  # Set left wheel direction to backwards
            #self.motor.set_wheels_speed(left=-(self.gain - self.trim)*self.goal_left_speed, right=(self.gain + self.trim)*self.goal_right_speed)
            desired_left = (self.gain - self.trim)*self.goal_left_speed
            desired_right = (self.gain + self.trim)*self.goal_right_speed
            left_cmd, right_cmd = self.acceleration_func(desired_left, desired_right) # Apply acceleration limits
            self.motor.set_wheels_speed(left=left_cmd, right=right_cmd) # Set the wheel speeds

        # === State 12: only command right wheel to move forwards  ===
        if self.state == 12:
            if self.prev_state != self.state:
                rospy.loginfo("State 12: command right wheel forwards")
                self.prev_state = self.state
                # update right wheel direction
                self.call_right_wheel_dir(1)  # Set right wheel direction to forwards
            #self.motor.set_wheels_speed(left=(self.gain - self.trim)*self.goal_left_speed, right=(self.gain + self.trim)*self.goal_right_speed)
            desired_left = (self.gain - self.trim)*self.goal_left_speed
            desired_right = (self.gain + self.trim)*self.goal_right_speed
            left_cmd, right_cmd = self.acceleration_func(desired_left, desired_right) # Apply acceleration limits
            self.motor.set_wheels_speed(left=left_cmd, right=right_cmd) # Set the wheel speeds
        
        # === State 13: only command right wheel to move backwars  ===
        if self.state == 13:
            if self.prev_state != self.state:
                rospy.loginfo("State 13: command right wheel backwards")
                self.prev_state = self.state
                # update right wheel direction
                self.call_right_wheel_dir(-1)  # Set right wheel direction to backwards
            #self.motor.set_wheels_speed(left=(self.gain - self.trim)*self.goal_left_speed, right=-(self.gain + self.trim)*self.goal_right_speed)
            desired_left = (self.gain - self.trim)*self.goal_left_speed
            desired_right = (self.gain + self.trim)*self.goal_right_speed
            left_cmd, right_cmd = self.acceleration_func(desired_left, desired_right) # Apply acceleration limits
            self.motor.set_wheels_speed(left=left_cmd, right=right_cmd) # Set the wheel speeds



    def pid_heading_control(self, v_nom, theta_ref, theta_hat):
        """
        PID heading controller.
        v_nom      : desired linear speed  (m/s; positive forward, negative back)
        theta_ref  : desired heading (rad)
        theta_hat  : current heading (rad)
        returns (v_nom, omega)  where omega is angular speed (rad/s)
        """
        now      = rospy.Time.now()
        dt       = (now - self.last_pid_time).to_sec()       # delta time since last PID call
        dt       = max(dt, 1e-3)                        # avoid division by zero
        self.last_pid_time = now                             # update last PID time

        # ----- error terms ---------------------------------------------------
        theta_error   = self.angle_diff(theta_ref, theta_hat)     # signed error
        de  = (theta_error - self.prev_theta_error) / dt          # derivative
        self.integral_error += theta_error * dt                   # integral (grows unbounded, so we need to clip it or reset it)
        #self.integral_error = np.clip(self.integral_error, -I_MAX, I_MAX) # or we can reset the integral error if the state changes

        # ----- PID -----------------------------------------------------------
        omega = (self.Kp * theta_error + self.Kd * de + self.Ki * self.integral_error) 

        self.prev_theta_error = theta_error    # update previous error for next iteration
        return v_nom, omega

    def v_omega_to_motor_cmd(self, v, omega):
        """
        Convert desired (v, omega) to left/right wheel commands in [-1, 1].
        Simple differential drive model: v = (v_r + v_l)/2, omega = (v_r - v_l)/baseline
        """
        v_r = v + 0.5 * omega * self.baseline
        v_l = v - 0.5 * omega * self.baseline

        # normalise by a nominal max wheel speed (m/s) that maps to PWM = 1
        right_cmd = np.clip(v_r / self.max_velocity, -1.0, 1.0)
        left_cmd  = np.clip(v_l / self.max_velocity, -1.0, 1.0)
        return left_cmd, right_cmd

    def acceleration_func(self, desired_left, desired_right):
        """
        Limit how fast wheel commands change to fight wheel slip.
        Inputs & outputs are in the same normalized range [-1, 1].
        """
        now  = rospy.Time.now()

        # Initialize last_cmd_time on first call
        if not hasattr(self, 'last_cmd_time') or self.last_cmd_time is None:
            self.last_cmd_time = now
            dt = 0.02  # assume first run is 20ms
        else:
            dt = (now - self.last_cmd_time).to_sec()
            dt = max(dt, 1e-3)  # Avoid zero or tiny dt values
        
        self.last_cmd_time = now  # Update timestamp for next call
        # dt = 0.02 # statically set the time step to 20ms (50Hz)

        def limit(prev, desired):
            limit_up   = self.acceleration * dt
            limit_down = self.deceleration * dt
            delta = desired - prev
            if   delta >  limit_up:
                delta =  limit_up
            elif delta < -limit_down:
                delta = -limit_down
            return np.clip(prev + delta, -1.0, 1.0)

        self.cmd_left  = limit(self.cmd_left,  desired_left) 
        self.cmd_right = limit(self.cmd_right, desired_right) 
        rospy.loginfo_throttle(1, "accel: dt=%.3f  desL=%.2f  prevL=%.2f  outL=%.2f  limit_up=%.3f", dt, desired_left, self.cmd_left, self.cmd_left, self.acceleration*dt)

        return self.cmd_left, self.cmd_right
                

    def load_param(self):
        config = {}
        config["gain"] = rospy.get_param("~gain")
        config["trim"] = rospy.get_param("~trim")
        config["baseline"] = rospy.get_param("~baseline")
        config["wheel_rad"] = rospy.get_param("~wheel_rad")
        config["encoder_res"] = rospy.get_param("~encoder_res")
        config["Kp"] = rospy.get_param("~Kp")
        config["Ki"] = rospy.get_param("~Ki")
        config["Kd"] = rospy.get_param("~Kd")
        config["max_velocity"] = rospy.get_param("~max_velocity")
        config["acceleration"] = rospy.get_param("~acceleration")
        config["deceleration"] = rospy.get_param("~deceleration")
        
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
