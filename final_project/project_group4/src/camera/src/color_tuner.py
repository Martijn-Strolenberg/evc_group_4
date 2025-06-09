#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge


class HSVTuner:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image_proc", CompressedImage, self.image_callback)

        # Trackbar window setup
        cv2.namedWindow("Trackbars")
        cv2.createTrackbar("LH", "Trackbars", 0, 179, self.nothing)
        cv2.createTrackbar("LS", "Trackbars", 0, 255, self.nothing)
        cv2.createTrackbar("LV", "Trackbars", 0, 255, self.nothing)
        cv2.createTrackbar("UH", "Trackbars", 179, 179, self.nothing)
        cv2.createTrackbar("US", "Trackbars", 255, 255, self.nothing)
        cv2.createTrackbar("UV", "Trackbars", 255, 255, self.nothing)

        self.latest_frame = None

    def nothing(self, x):
        pass

    def image_callback(self, msg):
        try:
            #self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        except Exception as e:
            rospy.logerr("CV Bridge error: %s", str(e))

    def run(self):
        rospy.loginfo("HSV Tuner Node Running...")
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.latest_frame is None:
                rate.sleep()
                continue

            frame = self.latest_frame.copy()
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Get current trackbar positions
            lh = cv2.getTrackbarPos("LH", "Trackbars")
            ls = cv2.getTrackbarPos("LS", "Trackbars")
            lv = cv2.getTrackbarPos("LV", "Trackbars")
            uh = cv2.getTrackbarPos("UH", "Trackbars")
            us = cv2.getTrackbarPos("US", "Trackbars")
            uv = cv2.getTrackbarPos("UV", "Trackbars")

            lower = np.array([lh, ls, lv])
            upper = np.array([uh, us, uv])

            mask = cv2.inRange(hsv, lower, upper)
            result = cv2.bitwise_and(frame, frame, mask=mask)

            cv2.imshow("Original", frame)
            cv2.imshow("Mask", mask)
            cv2.imshow("Filtered", result)

            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC to print and exit
                rospy.loginfo(f"Lower HSV: {lower.tolist()}")
                rospy.loginfo(f"Upper HSV: {upper.tolist()}")
                break

        cv2.destroyAllWindows()


if __name__ == '__main__':
    rospy.init_node("hsv_tuner_node")
    tuner = HSVTuner()
    tuner.run()
