#!/usr/bin/env python2

import rospy
import numpy as np
from motor_GUI import MotorGUI
from motor_control.msg import motor_cmd

# USER DEFINED SPEEDS
def_velocity = 1    # Velocity in rad / sec
def_distance = 1    # Distance in m
def_angle_deg = 1   # angle in degrees

class MotorPublisherNode:
    def __init__(self):
        self.initialized = False
        rospy.loginfo("Initializing motor node...")

        # Init Publisher
        self.pub_cmd = rospy.Publisher(
            "/motor_control",
            motor_cmd,
            queue_size=10
        )
        self.motor_gui = MotorGUI()
        # override self.motor_gui.submit to run publish_msg
        self.motor_gui.submit = self.publish_msg
    
        
    def publish_msg(self):

        msg = motor_cmd()


        msg.velocity.data = self.motor_gui.velocity
        msg.position.data = self.motor_gui.position
        msg.angle.data = self.motor_gui.angle

        self.pub_cmd = rospy.publish(msg)

    def cleanup(self):
        cv2.destroyAllWindows()

# Main function run the node
if __name__ == "__main__":
    # Initialize the nodes
    rospy.init_node('motor_pub_node', anonymous=True, xmlrpc_port=45100, tcpros_port=45101)
    motor_node = MotorPublisherNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down motor cmd publisher.")
    #finally:
    #    camera_node.cleanup()


