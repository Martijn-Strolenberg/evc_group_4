#!/usr/bin/env python3

import rospy
from test_pkgs.srv import Stop

def stop_robot_client():
    rospy.wait_for_service('stop_robot')
    try:
        stop_robot = rospy.ServiceProxy('stop_robot', Stop)
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