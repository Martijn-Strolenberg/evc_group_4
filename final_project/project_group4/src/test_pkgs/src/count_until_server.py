#!/usr/bin/env python

import rospy
import actionlib
from motor_control.msg import CountUntilAction, CountUntilFeedback, CountUntilResult

class CountUntilServer:
  def __init__(self):
    self.server = actionlib.SimpleActionServer(
      'count_goal',
      CountUntilAction,
      self.execute,
      False
    )
    self.server.start()
    rospy.loginfo("CountUntil Action Server Started.")

  def execute(self, goal):
    rate = rospy.Rate(1)  # 1 Hz
    success = True
    feedback = CountUntilFeedback()

    for i in range(goal.count_goal + 1):
      if self.server.is_preempt_requested():
        rospy.loginfo("Preempted!")
        self.server.set_preempted()
        success = False
        break

      feedback.current_number = i
      self.server.publish_feedback(feedback)
      rospy.loginfo("Counting: {}".format(i))
      rate.sleep()

    result = CountUntilResult(success=success)
    self.server.set_succeeded(result)

if __name__ == '__main__':
  rospy.init_node('count_until_server')
  server = CountUntilServer()
  rospy.spin()