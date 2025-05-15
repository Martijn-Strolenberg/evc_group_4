#!/usr/bin/env python2

import rospy
import numpy as np
from motor_control.msg import motor_cmd

# USER DEFINED SPEEDS
def_velocity = 1    # Velocity in rad / sec
def_distance = 1    # Distance in m
def_angle_deg = 1   # angle in degrees

class MotorPublisherNode:
    def __init__(self):
        self.initialized = False
        rospy.loginfo("Initializing motor publisher node...")

        # Init Publisher
        self.pub_cmd = rospy.Publisher(
            "/motor_control",
            motor_cmd,
            queue_size=10
        )
        self.new_cmd = 0 
        # self.motor_gui = MotorGUI()
        # override self.motor_gui.submit to run publish_msg
        # self.motor_gui.submit = self.publish_msg

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        try:
            while not rospy.is_shutdown():
                try:
                    # Read velocity input
                    v_input = raw_input("Enter velocity (m/s) [or 'q' to quit]: ")
                    if v_input.strip().lower() == 'q':
                        rospy.loginfo("Exiting motor command publisher.")
                        break
                    velocity = float(v_input)

                    # Read position input
                    p_input = raw_input("Enter position (m): ")
                    if p_input.strip().lower() == 'q':
                        rospy.loginfo("Exiting motor command publisher.")
                        break
                    position = float(p_input)

                    # Read angle input
                    a_input = raw_input("Enter angle (degrees): ")
                    if a_input.strip().lower() == 'q':
                        rospy.loginfo("Exiting motor command publisher.")
                        break
                    angle = float(a_input)

                    self.new_cmd = self.new_cmd + 1

                    msg = motor_cmd()
                    msg.velocity = velocity
                    msg.distance = position
                    msg.angle = np.deg2rad(angle)
                    msg.new_mesg = self.new_cmd
                    

                    rospy.loginfo("Publishing motor command: v=%.2f, p=%.2f, a=%.2f", velocity, position, angle)
                    self.pub_cmd.publish(msg)

                except ValueError:
                    rospy.logwarn("Invalid input. Please enter numeric values.")
                except rospy.ROSInterruptException:
                    break

                rate.sleep()
        except (rospy.ROSInterruptException, KeyboardInterrupt):
            rospy.loginfo("Shutting down motor cmd publisher.")
    
        
    def publish_msg(self): # not used anymore

        msg = motor_cmd()


        msg.velocity.data = self.motor_gui.velocity
        msg.position.data = self.motor_gui.position
        msg.angle.data = self.motor_gui.angle

        self.pub_cmd = rospy.publish(msg)

# Main function run the node
if __name__ == "__main__":
    # Initialize the nodes
    rospy.init_node('motor_pub_node')
    motor_node = MotorPublisherNode()
    motor_node.run()
