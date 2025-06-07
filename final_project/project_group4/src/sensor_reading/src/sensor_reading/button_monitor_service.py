#!/usr/bin/env python2

import rospy
from sensor_reading.srv import ButtonPressed
from sensor_reading.drivers.buttonClass import ButtonEvent, ButtonDriver
import threading

LED_GPIO = 37
SIGNAL_GPIO = 40

class ButtonMonitorNode:
    def __init__(self):
        self.ledState = 1
        self.driver = None
        rospy.loginfo("Service button_pressed is ready.")

        # Initialize button driver
        self.driver = ButtonDriver(LED_GPIO, SIGNAL_GPIO, self.threaded_cb)
        self.driver.led.set(self.ledState)

    def threaded_cb(self, event):
        # Run actual event handler in new thread
        threading.Thread(target=self.event_cb, args=(event,)).start()

    def call_button_update(self, pressed):
        rospy.loginfo("h1")
        rospy.wait_for_service('button_pressed')
        rospy.loginfo("h2")
        try:
            proxy = rospy.ServiceProxy('button_pressed', ButtonPressed)
            resp = proxy(pressed)
            print("button update success:", resp.success)
        except rospy.ServiceException as e:
            print("Service call failed:", e)

    def event_cb(self, event):
        if event == ButtonEvent.PRESS:
            rospy.loginfo("Button pressed.")
            self.call_button_update(1)
        elif event == ButtonEvent.RELEASE:
            rospy.loginfo("Button released. Toggling LED.")
            self.ledState ^= 1
            self.driver.led.set(self.ledState)
            self.call_button_update(0)
        else:
            rospy.logwarn("Unknown button event: %s", str(event))
            return
 
def main():
    rospy.init_node('button_pressed_monitor')
    ButtonMonitorNode()
    rospy.loginfo("Button pressed monitor node started.")

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    main()