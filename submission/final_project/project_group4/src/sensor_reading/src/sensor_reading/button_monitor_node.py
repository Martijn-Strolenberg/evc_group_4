#!/usr/bin/env python2

import rospy
from sensor_reading.srv import ButtonPressed
from sensor_reading.drivers.buttonClass import ButtonEvent, ButtonDriver
from std_msgs.msg import Int8
import threading
import time

LED_GPIO = 37
SIGNAL_GPIO = 40
DOUBLE_TAP_THRESHOLD = 1  # seconds

class ButtonMonitorNode:
    def __init__(self):
        self.ledState = 1
        # Initialize button driver
        self.driver = ButtonDriver(LED_GPIO, SIGNAL_GPIO, self.threaded_cb)
        self.driver.led.set(self.ledState)
    
        self.button_pub = rospy.Publisher('/button_state', Int8, queue_size=10)

        self.last_press_time = None
        self.tap_count = 0
        self.tap_timer = None
        self.lock = threading.Lock()


    def threaded_cb(self, event):
        # Run actual event handler in new thread
        threading.Thread(target=self.event_cb, args=(event,)).start()

    def call_button_update(self, pressed):
        rospy.wait_for_service('button_pressed')
        try:
            proxy = rospy.ServiceProxy('button_pressed', ButtonPressed)
            resp = proxy(pressed)
            print("button update success:", resp.success)
        except rospy.ServiceException as e:
            print("Service call failed:", e)

    def event_cb(self, event):
        if event == ButtonEvent.PRESS:
            rospy.loginfo("Button pressed.")
        elif event == ButtonEvent.RELEASE:
            rospy.loginfo("Button released. Toggling LED.")
            self.ledState ^= 1
            self.driver.led.set(self.ledState)
            self.register_tap()
        else:
            rospy.logwarn("Unknown button event: %s", str(event))
            return
        
    def register_tap(self):
        with self.lock:
            now = time.time()
            if self.last_press_time and (now - self.last_press_time) <= DOUBLE_TAP_THRESHOLD:
                self.tap_count += 1
            else:
                self.tap_count = 1

            self.last_press_time = now

            if self.tap_timer:
                self.tap_timer.cancel()

            self.tap_timer = threading.Timer(DOUBLE_TAP_THRESHOLD, self.evaluate_taps)
            self.tap_timer.start()
        
    def evaluate_taps(self):
        with self.lock:
            if self.tap_count == 1:
                rospy.loginfo("Single tap detected.")
                self.publish_button_state(1)
            elif self.tap_count == 2:
                rospy.loginfo("Double tap detected.")
                self.publish_button_state(2)
            else:
                rospy.loginfo("Multi-tap (%d) detected.", self.tap_count)
                self.publish_button_state(self.tap_count)

            self.tap_count = 0
            self.last_press_time = None

    def publish_button_state(self, tap_type):
        msg = Int8()
        msg.data = tap_type
        self.button_pub.publish(msg)
        rospy.loginfo("Published button state: %s", tap_type)

        # # Call the service to notify other nodes
        # try:
        #     resp = self.service_proxy(pressed_value)
        #     if resp.success:
        #         rospy.loginfo("Service call succeeded.")
        #     else:
        #         rospy.logwarn("Service call returned failure.")
        # except rospy.ServiceException as e:
        #     rospy.logerr("Service call failed: %s" % e)

def main():
    rospy.init_node('button_pressed_monitor')
    node = ButtonMonitorNode()
    rospy.loginfo("Button pressed monitor node started.")

    rospy.spin()
    # rate = rospy.Rate(10)
    # while not rospy.is_shutdown():
    #     rate.sleep()

if __name__ == '__main__':
    main()