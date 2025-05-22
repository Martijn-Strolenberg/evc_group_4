#!/usr/bin/env python

import rospy
from test_pkgs.srv import StopRobot

def stop_robot_client():
    rospy.wait_for_service('stop_robot')
    try:
        stop_robot = rospy.ServiceProxy('stop_robot', StopRobot)
        resp = stop_robot()
        if resp.success:
            rospy.loginfo("Robot successfully stopped.")
        else:
            rospy.logwarn("Failed to stop robot.")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

if __name__ == "__main__":
    rospy.init_node('stop_robot_client')
    stop_robot_client()