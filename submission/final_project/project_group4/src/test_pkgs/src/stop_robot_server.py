#!/usr/bin/env python3

import rospy
from test_pkgs.srv import Stop, StopResponse
from geometry_msgs.msg import Twist

def handle_stop_robot(req):
    stop_msg = Twist()
    stop_msg.linear.x = 0.0
    stop_msg.angular.z = 0.0
    pub.publish(stop_msg)
    rospy.loginfo("Stop command sent to robot.")
    return StopResponse(success=True)

def stop_robot_server():
    global pub
    rospy.init_node('stop_robot_server', anonymous=False, xmlrpc_port=45104, tcpros_port=45105)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Service('stop_robot', Stop, handle_stop_robot)
    rospy.loginfo("StopRobot service ready.")
    rospy.spin()

if __name__ == "__main__":
    stop_robot_server()