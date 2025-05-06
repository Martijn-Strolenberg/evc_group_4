#!/usr/bin/env python2

import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

THRESH = 110

def on_trackbar(val):
    global THRESH
    THRESH = val

class CameraSubscriberNode:
    def __init__(self):
        self.initialized = False
        rospy.loginfo("Initializing camera subscriber node...")
        
        self.topic_name = '/camera/image_raw'
        #self.window_name = "Camera View"
        # windows
        cv2.namedWindow("raw",      cv2.WINDOW_NORMAL)
        cv2.namedWindow("Mask",     cv2.WINDOW_NORMAL)
        cv2.namedWindow("Orange",cv2.WINDOW_NORMAL)
        cv2.createTrackbar("Thresh", "threshold", THRESH, 255, on_trackbar)

        self.bridge = CvBridge()
        
        # Construct subscriber
        self.sub_image = rospy.Subscriber(
            self.topic_name,
            Image,
            self.image_cb,
            queue_size=1
        )

        #cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        self.initialized = True
        rospy.loginfo("Camera subscriber node initialized!")

    def image_cb(self, data):
        if not self.initialized:
            return
        
        try:
            cv_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: %s", e)
            return

        # Process the image
        # detect & draw
        box = self.detect_card(cv_img)
        if box is not None:
           cv2.polylines(cv_img, [box], True, (0,255,0), 2)  

        # Show raw
        cv2.imshow("raw", cv_img)

        # 2) BGR to HSV
        hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

        # 3) Threshold for orange
        lower = np.array([ 10, 60, 60])
        upper = np.array([ 25, 255, 255])
        mask  = cv2.inRange(hsv, lower, upper)

        # 4) Clean up
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        # 5) Find contours
        res = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = res[-2]  # works in both OpenCV3 & 4

        if cnts:
            # pick largest
            c = max(cnts, key=cv2.contourArea)
            if cv2.contourArea(c) > 500:  # ignore small specks
                # 6) Compute centroid
                M = cv2.moments(c)
                if M['m00'] != 0:
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    # 7) Draw it
                    cv2.circle(cv_img, (cx,cy), 10, (0,255,0), -1)
                    cv2.drawContours(cv_img, [c], -1, (0,255,0), 2)

        cv2.imshow("Orange", cv_img)
        cv2.imshow("Mask", mask)

        # waitKey to update windows
        cv2.waitKey(1)      

        # Display the image
        #cv2.imshow(self.window_name, cv_img)
        cv2.waitKey(1) # wait for 1 ms

    def detect_card(self, frame):
        h, w = frame.shape[:2]
        # 1) grayscale + blur
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (3,3), 0)

        # 2) threshold for white regions
        _, th = cv2.threshold(blur, 120, 255, cv2.THRESH_BINARY)

        # 3) find contours
        # OpenCV3 vs 4 unpacking
        res = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = res[-2]
        if not cnts: 
            return None

        # 4) look for the best quadrilateral
        best = None
        max_area = 0
        for c in cnts:
            area = cv2.contourArea(c)
            if area < 10000:  # skip small blobs
                continue
            # approximate to polygon
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.02*peri, True)
            if len(approx) == 4 and area > max_area:
                # check aspect ratio
                pts = approx.reshape(4,2).astype(np.float32)
                # compute width/height via distances
                d01 = np.linalg.norm(pts[0]-pts[1])
                d12 = np.linalg.norm(pts[1]-pts[2])
                ar = max(d01, d12) / (min(d01, d12) + 1e-5)
                if 1.4 < ar < 1.8:  # credit card ~1.585
                    best = approx
                    max_area = area

        return best  # either None or a (4,1,2) array of points

    def cleanup(self):
        rospy.loginfo("Shutting down camera subscriber, closing window.")
        cv2.destroyAllWindows()

if __name__ == "__main__":
    # Initialize the node
    rospy.init_node('camera_subscriber_node', anonymous=True)
    camera_node = CameraSubscriberNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down image viewer node.")
    finally:
        camera_node.cleanup()
