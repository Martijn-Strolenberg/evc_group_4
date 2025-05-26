#!/usr/bin/env python

import rospy
from test_pkgs.srv import MoveStraight, Rotate, Stop

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
  print("rotate <angle> <angular_speed>")
  print("stop\n")

  while not rospy.is_shutdown():
    try:
      user_input = raw_input(">>> ").strip()
      if not user_input:
        continue
      parts = user_input.split()
      cmd = parts[0]

      if cmd == "move_straight" and len(parts) == 3:
        call_move_straight(float(parts[1]), float(parts[2]))
      elif cmd == "rotate" and len(parts) == 3:
        call_rotate(float(parts[1]), float(parts[2]))
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
  