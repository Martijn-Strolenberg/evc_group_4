#!/usr/bin/env python

import rospy
import actionlib
import sys
from motor_control.msg import CountUntilAction, CountUntilGoal

def feedback_cb(feedback):
    rospy.loginfo("Current number: %d", feedback.current_number)

def count_until_client(count_target):
    rospy.init_node('count_until_client')

    client = actionlib.SimpleActionClient('count_goal', CountUntilAction)
    rospy.loginfo("Waiting for action server...")
    client.wait_for_server()

    goal = CountUntilGoal(count_goal=count_target)
    rospy.loginfo("Sending goal to count until %d...", count_target)
    client.send_goal(goal, feedback_cb=feedback_cb)

    client.wait_for_result()
    result = client.get_result()
    if result.success:
      rospy.loginfo("Counting completed successfully.")
    else:
      rospy.logwarn("Counting failed or was preempted.")

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage: rosrun motor_control count_until_client.py <number>")
        sys.exit(1)

    try:
        target = int(sys.argv[1])
        count_until_client(target)
    except ValueError:
        print("Invalid number: {}".format(sys.argv[1]))