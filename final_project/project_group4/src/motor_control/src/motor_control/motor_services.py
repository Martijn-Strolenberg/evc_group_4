#!/usr/bin/env python2

import rospy
from motor_control.srv import (          # import all ros services
    MoveStraight, MoveStraightResponse,
    Rotate, RotateResponse,
    Stop, StopResponse,
    ConstRotate, ConstRotateResponse,
    ConstStraight, ConstStraightResponse,
    LeftWheelDir, RightWheelDir
)

class MotorServices(object):
    """
    All service endpoints live here.  The callbacks *delegate* to the
    `controller` instance that actually drives the motors & state-machine.
    """

    def __init__(self, controller):
        self.ctrl = controller    # keep a reference

        # ------------- service registrations -------------
        self._move_srv   = rospy.Service('move_straight',  MoveStraight,  self._move_cb)
        self._rot_srv    = rospy.Service('rotate',         Rotate,        self._rot_cb)
        self._stop_srv   = rospy.Service('stop',           Stop,          self._stop_cb)
        self._crot_srv   = rospy.Service('const_rotate',   ConstRotate,   self._crot_cb)
        self._cstr_srv   = rospy.Service('const_straight', ConstStraight, self._cstr_cb)
        # direction services are “client-side helpers” – keep them if you still need them
        self._lwd_srv    = rospy.Service('left_wheel_dir',  LeftWheelDir,  self._lwd_cb)
        self._rwd_srv    = rospy.Service('right_wheel_dir', RightWheelDir, self._rwd_cb)

    # ---------- callbacks ----------
    def _move_cb(self, req):
        ok = self.ctrl.start_straight(req.distance, req.speed)
        return MoveStraightResponse(success=ok)

    def _rot_cb(self, req):
        ok = self.ctrl.start_rotate(req.angle, req.angular_speed)
        return RotateResponse(success=ok)

    def _stop_cb(self, _req):
        self.ctrl.stop()
        return StopResponse(success=True)

    def _crot_cb(self, req):
        ok = self.ctrl.start_const_rotate(req.direction, req.angular_speed)
        return ConstRotateResponse(success=ok)

    def _cstr_cb(self, req):
        ok = self.ctrl.start_const_straight(req.direction, req.speed)
        return ConstStraightResponse(success=ok)

    # ----------- wheel-direction services ------------
    def _lwd_cb(self, req):
        return self.ctrl.set_left_dir(req.direction)

    def _rwd_cb(self, req):
        return self.ctrl.set_right_dir(req.direction)