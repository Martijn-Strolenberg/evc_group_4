#!/usr/bin/env python

import rospy, math, time
from geometry_msgs.msg import Twist
from test_pkgs.srv import MoveStraight, MoveStraightResponse
from test_pkgs.srv import Rotate, RotateResponse
from test_pkgs.srv import Stop, StopResponse

class MotionAPI(object):
  def __init__(self):
    rospy.init_node('motion_api_node')
    self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    rospy.Service('move_straight', MoveStraight, self.handle_move_straight)
    rospy.Service('rotate', Rotate, self.handle_rotate)
    rospy.Service('stop', Stop, self.handle_stop)

    rospy.loginfo("Motion API services ready.")
    rospy.spin()

  # ---------- service handlers ----------
  def handle_move_straight(self, req):
    if req.speed <= 0:
      rospy.logwarn("Speed must be > 0")
      return MoveStraightResponse(False)

    duration = abs(req.distance) / req.speed
    rospy.loginfo("MOVING straight: %.2fm at %.2fm/s (%.2fs)", req.distance, req.speed, duration)

    # twist = Twist()
    # twist.linear.x = math.copysign(req.speed, req.distance)
    # self._publish_for_duration(twist, duration)
    # self._stop_robot()

    return MoveStraightResponse(True)

  def handle_rotate(self, req):
    if req.angular_speed <= 0:
      rospy.logwarn("Angular speed must be >0")
      return RotateResponse(False)

    duration = abs(req.angle) / req.angular_speed
    rospy.loginfo("ROTATING: %.2frad at %.2frad/s (%.2fs)", req.angle, req.angular_speed, duration)

    # twist = Twist()
    # twist.angular.z = math.copysign(req.angular_speed, req.angle)
    # self._publish_for_duration(twist, duration)
    # self._stop_robot()
    
    return RotateResponse(True)

  def handle_stop(self, req):
    # self._stop_robot()
    rospy.loginfo("STOP command received.")
    return StopResponse(True)

  # ---------- helpers ----------
  def _publish_for_duration(self, twist, duration):
    rate = rospy.Rate(20)
    end_time = rospy.Time.now() + rospy.Duration(duration)
    while rospy.Time.now() < end_time and not rospy.is_shutdown():
      self.pub.publish(twist)
      rate.sleep()

  def _stop_robot(self):
    self.pub.publish(Twist())  # zero velocities
    rospy.loginfo("Robot stopped.")

if __name__ == '__main__':
  MotionAPI()
