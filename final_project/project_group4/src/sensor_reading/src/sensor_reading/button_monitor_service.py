#!/usr/bin/env python2

import rospy
from sensor_reading.srv import ButtonPressed, ButtonPressedResponse
from buttonClass import ButtonEvent, ButtonDriver

LED_GPIO = 37
SIGNAL_GPIO = 40

class ButtonNode:
    def __init__(self):
        self.ledState = 1
        self.service_proxy = rospy.ServiceProxy('button_pressed', ButtonPressed)

        # Initialize button driver
        self.driver = ButtonDriver(LED_GPIO, SIGNAL_GPIO, self.event_cb)
        self.driver.led.set(self.ledState)

        # Wait for the service to become available
        rospy.wait_for_service('button_pressed')
        rospy.loginfo("Service button_pressed is ready.")

    def event_cb(self, event):
        if event == ButtonEvent.PRESS:
            rospy.loginfo("Button pressed.")
            return

        if event == ButtonEvent.RELEASE:
            rospy.loginfo("Button released. Toggling LED.")
            self.ledState ^= 1
            self.driver.led.set(self.ledState)

            # Call the service to notify other nodes
            try:
                resp = self.service_proxy()
                if resp.success:
                    rospy.loginfo("Service call succeeded.")
                else:
                    rospy.logwarn("Service call returned failure.")
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s" % e)

def main():
    rospy.init_node('button_pressed_monitor')
    button_node = ButtonNode()

    rospy.loginfo("Button pressed monitor node started.")
    rospy.spin()

if __name__ == '__main__':
    main()