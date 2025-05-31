#!/usr/bin/env python

import rospy
import numpy as np
from motor_control.srv import MoveStraight, Rotate, Stop, ConstRotate, ConstStraight, DriveLeftwheel, DriveRightwheel

def call_left_wheel(direction, speed):
  rospy.wait_for_service('left_wheel_vel')
  try:
    proxy = rospy.ServiceProxy('left_wheel_vel', DriveLeftwheel)
    resp = proxy(direction, speed)
    print("Left wheel command success:", resp.success)
  except rospy.ServiceException as e:
    print("Service call failed:", e)


def call_right_wheel(direction, speed):
  rospy.wait_for_service('right_wheel_vel')
  try:
    proxy = rospy.ServiceProxy('right_wheel_vel', DriveRightwheel)
    resp = proxy(direction, speed)
    print("Right wheel command success:", resp.success)
  except rospy.ServiceException as e:
    print("Service call failed:", e)

def call_const_rotate(direction, angular_speed):
  rospy.wait_for_service('const_rotate')
  try:
    proxy = rospy.ServiceProxy('const_rotate', ConstRotate)
    resp = proxy(direction, angular_speed)
    print("Constant Rotate success:", resp.success)
  except rospy.ServiceException as e:
    print("Service call failed:", e)

def call_const_straight(direction, speed):
  rospy.wait_for_service('const_straight')
  try:
    proxy = rospy.ServiceProxy('const_straight', ConstStraight)
    resp = proxy(direction, speed)
    print("Moving Constant Straight success:", resp.success)
  except rospy.ServiceException as e:
    print("Service call failed:", e)

def call_move_straight(distance, speed):
  rospy.wait_for_service('move_straight')
  try:
    proxy = rospy.ServiceProxy('move_straight', MoveStraight)
    resp = proxy(distance, speed)
    print("MoveStraight success:", resp.success)
  except rospy.ServiceException as e:
    print("Service call failed:", e)

def call_rotate(angle, angular_speed):
  rospy.wait_for_service('rotate')
  try:
    proxy = rospy.ServiceProxy('rotate', Rotate)
    resp = proxy(angle, angular_speed)
    print("Rotate success:", resp.success)
  except rospy.ServiceException as e:
    print("Service call failed:", e)

def call_stop():
  rospy.wait_for_service('stop')
  try:
    proxy = rospy.ServiceProxy('stop', Stop)
    resp = proxy()
    print("Stop success:", resp.success)
  except rospy.ServiceException as e:
    print("Service call failed:", e)

def interactive_prompt():
  print("\nMotion Command Interface (Ctrl+C to quit)")
  print("Commands:")
  print("move_straight <distance> <speed>")
  print("rotate <angle (deg)> <angular_speed>")
  print("const_rotate <direction> <speed>")
  print("const_straight <direction> <speed>")
  print("left_wheel <direction> <speed>")
  print("right_wheel <direction> <speed>")
  print("stop\n")

  while not rospy.is_shutdown():
    try:
      user_input = raw_input(">>> ").strip()
      if not user_input:
        continue
      parts = user_input.split()
      cmd = parts[0]

      if cmd == "left_wheel" and len(parts) == 3:
        call_left_wheel(float(parts[1]), float(parts[2]))
      if cmd == "right_wheel" and len(parts) == 3:
        call_right_wheel(float(parts[1]), float(parts[2]))
      if cmd == "const_rotate" and len(parts) == 3:
        call_const_rotate(float(parts[1]), float(parts[2]))
      if cmd == "const_straight" and len(parts) == 3:
        call_const_straight(float(parts[1]), float(parts[2]))
      if cmd == "move_straight" and len(parts) == 3:
        call_move_straight(float(parts[1]), float(parts[2]))
      elif cmd == "rotate" and len(parts) == 3:
        call_rotate(np.deg2rad(float(parts[1])), float(parts[2]))
      elif cmd == "stop":
        call_stop()
      else:
        print("Invalid command or arguments.")
    except KeyboardInterrupt:
      print("\nExiting.")
      break
    except Exception as e:
      print("Error:", e)

if __name__ == "__main__":
  rospy.init_node("motion_client_interactive")
  interactive_prompt()
  