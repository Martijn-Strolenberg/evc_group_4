#!/usr/bin/env python2

import rospy
import numpy as np
from typing import Tuple
from motor_control.drivers.encoderDriver import *
# from encoderDriver import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
import tf
import tf.transformations as tft
from motor_control.srv import LeftWheelDir, LeftWheelDirResponse
from motor_control.srv import RightWheelDir, RightWheelDirResponse

class OdometryPublisherNode:
  def __init__(self):
    self.initialized = False
    rospy.loginfo("Initializing odometry node...")

    # Hardware for encoders
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
    self.ticks_L = 0          # current left ticks
    self.prev_ticks_L = 0     # previous left ticks
    self.ticks_R = 0          # current right ticks
    self.prev_ticks_R = 0     # previous right ticks
    #self.alpha = 2 * np.pi / self.encoder_resolution  # radians per tick? used in delta_phi function so this is not needed here
    self.delta_phi_left = 0   # rotation of the left wheel in radians
    self.delta_phi_right = 0  # rotation of the right wheel in radians

    ## Parameters for odomemtry (pose estimation)
    self.x_prev = 0           # previous x estimate
    self.y_prev = 0           # previous y estimate
    self.theta_prev = 0       # previous orientation estimate
    self.x_curr = 0.0         # current x estimate
    self.y_curr = 0.0         # current y estimate
    self.theta_curr = 0.0     # current orientation estimate

    self.sample_period = 0.02 # seconds

    # start reading the encoders periodically
    self.timer = rospy.Timer(rospy.Duration(self.sample_period), self.read_encoder) # publishing at sample rate 50 Hz execute function read_encoder()
    self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10) # create publisher to publish odometry data to topic /odom
    self.tf_broadcaster = tf.TransformBroadcaster() # create a tf broadcaster
    rospy.Service('left_wheel_dir', LeftWheelDir, self.handle_left_wheel_dir)    # service to set the direction of the left wheel
    rospy.Service('right_wheel_dir', RightWheelDir, self.handle_right_wheel_dir) # service to set the direction of the right wheel
    rospy.on_shutdown(self.shutdown) # register shutdown function to clean up GPIO
    rospy.loginfo("Odometry node initialized!")
    self.initialized = True

  # ------------- service calls ------------- 
  def handle_left_wheel_dir(self, req):
    """ Service to set the direction of the left wheel."""
    if   req.direction == 1:
      self.driver_L.set_direction(WheelDirection.FORWARD)
    elif req.direction == -1:
      self.driver_L.set_direction(WheelDirection.REVERSE)
    else:
      rospy.logwarn("Invalid direction (-1 or 1 expected)")
      return LeftWheelDirResponse(False)
    return LeftWheelDirResponse(True)
    
  def handle_right_wheel_dir(self, req):
    """ Service to set the direction of the right wheel."""
    if   req.direction == 1:
      self.driver_R.set_direction(WheelDirection.FORWARD)
    elif req.direction == -1:
      self.driver_R.set_direction(WheelDirection.REVERSE)
    else:
      rospy.logwarn("Invalid direction (-1 or 1 expected)")
      return RightWheelDirResponse(False)
    return RightWheelDirResponse(True)

  # Function that gets called at the sample rate
  def read_encoder(self, event):
    # read the encoders
    # self.ticks_L = self.driver_L._ticks # read latest value of the left encoder
    # self.ticks_R = self.driver_R._ticks # read latest value of the right encoder
    self.ticks_L = self.driver_L.get_ticks()
    self.ticks_R = self.driver_R.get_ticks()

    # calculate the wheel rotation in the sample period
    self.delta_phi_left = self.delta_phi(self.ticks_L, self.prev_ticks_L)
    self.delta_phi_right = self.delta_phi(self.ticks_R, self.prev_ticks_R)
    # update the previous ticks
    self.prev_ticks_L = self.ticks_L
    self.prev_ticks_R = self.ticks_R

    # calculate the current pose estimation
    [self.x_curr, self.y_curr, self.theta_curr, d_average, d_theta] = self.pose_estimation(self.wheel_radius, self.baseline, self.x_prev, self.y_prev, self.theta_prev, self.delta_phi_left, self.delta_phi_right)
    # update the current pose estimation parameters
    self.x_prev = self.x_curr
    self.y_prev = self.y_curr
    self.theta_prev = self.theta_curr

    # recompute distances to use in odometry
    d_right = self.wheel_radius * self.delta_phi_right
    d_left = self.wheel_radius * self.delta_phi_left
    d_average = (d_right + d_left) / 2
    d_theta = (d_right - d_left) / self.baseline

    # publish the odometry data and the tf transform
    self.publish_odometry(self.x_curr, self.y_curr, self.theta_curr, d_average, d_theta)


  def delta_phi(self, ticks, prev_ticks):
    """
    Args:
      ticks: Current tick count from the encoders.
      prev_ticks: Previous tick count from the encoders.
    Return:
      delta_phi: Rotation of the wheel in radians.
    """
    # ---
    alpha = 2 * np.pi / self.encoder_resolution  # radians per tick
    delta_ticks = ticks - prev_ticks

    delta_phi = alpha * delta_ticks  # rotation of the wheel in radians

    return delta_phi

  def pose_estimation(self, R, baseline, x_prev, y_prev, theta_prev, delta_phi_left, delta_phi_right):
    """
    Calculate the current robot pose using the dead-reckoning model.

    Args:
      R:                  radius of wheel (both wheels are assumed to have the same size)
      baseline:           distance from wheel to wheel; 2L of the theory
      x_prev:             previous x estimate - assume given
      y_prev:             previous y estimate - assume given
      theta_prev:         previous orientation estimate - assume given
      delta_phi_left:     left wheel rotation (rad)
      delta_phi_right:    right wheel rotation (rad)

    Return:
      x_curr:                  estimated x coordinate
      y_curr:                  estimated y coordinate
      theta_curr:              estimated heading
    """
    # ---
    d_right = R*delta_phi_right  # distance travelled by right wheel
    d_left = R*delta_phi_left    # distance travelled by left wheel

    d_average = (d_right + d_left) / 2  # average distance travelled
    d_theta = (d_right - d_left) / baseline  # change in orientation

    d_x = d_average * np.cos(theta_prev + d_theta/2)  # change in x
    d_y = d_average * np.sin(theta_prev + d_theta/2)  # change in y ; d_y = d_average * np.sin(theta_prev + d_theta / 2)

    x_curr = x_prev + d_x  # new x coordinate
    y_curr = y_prev + d_y  # new y coordinate
    #theta_curr = theta_prev + d_theta  # new orientation
    theta_curr = (theta_prev + d_theta + np.pi) % (2*np.pi) - np.pi  # new orientation

    return x_curr, y_curr, theta_curr, d_average, d_theta

  # Function to publish the odometry data and the tf transform
  def publish_odometry(self, x, y, theta, d_average, d_theta):
    # time stamp
    now = rospy.Time.now()

    # quaternion from yaw
    quat = tft.quaternion_from_euler(0, 0, theta)

    # 1) TF -----------------------------------------------------------------
    self.tf_broadcaster.sendTransform(
      (x, y, 0.0),
      quat,
      now,
      "base_link",    # child frame
      "odom"          # parent frame
    )

    # 2) Odometry message ----------------------------------------------------
    odom = Odometry()
    odom.header.stamp = now
    odom.header.frame_id = "odom"
    odom.child_frame_id = "base_link"
    odom.pose.pose.position.x = x
    odom.pose.pose.position.y = y
    odom.pose.pose.orientation = Quaternion(*quat)
    odom.twist.twist.linear.x = d_average / self.sample_period  # linear velocity
    odom.twist.twist.angular.z = d_theta / self.sample_period  # angular velocity

    # (optional) set velocity if you compute it
    self.odom_pub.publish(odom)

  def load_param(self):
    rospy.loginfo("Loading parameters...")
    config = {}
    config["baseline"] = rospy.get_param("~baseline")
    config["wheel_rad"] = rospy.get_param("~wheel_rad")
    config["encoder_res"] = rospy.get_param("~encoder_res")
    config["velocity"] = rospy.get_param("~velocity")
    
    rospy.loginfo("Loaded config: %s", config)
    return config  

  def shutdown(self):
    rospy.loginfo("Shutting down: cleaning up GPIO...")
    self.driver_L.shutdown()
    self.driver_R.shutdown()
    GPIO.cleanup()  # cleanup GPIO pins


if __name__ == "__main__":
  # Initialize the node
  rospy.init_node('marti_odometry_node', anonymous=False, xmlrpc_port=45102, tcpros_port=45103)
  odometry_node = OdometryPublisherNode()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    rospy.loginfo("Shutting down marti odometry node.")
