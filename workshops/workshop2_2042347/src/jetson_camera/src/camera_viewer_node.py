#!/usr/bin/env python2

import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
from jetson_camera.msg import twovids


class CameraSubscriberNode:
    def __init__(self):
        self.initialized = False
        rospy.loginfo("Initializing camera subscriber node...")
        self.bridge = CvBridge()
        
        # Construct subscriber
        self.sub_image = rospy.Subscriber(
            "/camera/image_proc",
            twovids,
            self.image_cb,
            buff_size=2**24,
            queue_size=1
        )

        self.first_image_received = False
        self.initialized = True
        rospy.loginfo("Camera subscriber node initialized!")


    def image_cb(self, data):
        if not self.initialized:
            return
        
        if not self.first_image_received:
            self.first_image_received = True
            rospy.loginfo("Camera subscriber captured first image from publisher.")
        try:
            # Decode image without CvBridge
            raw_image = cv2.imdecode(np.frombuffer(data.raw_img.data, np.uint8), cv2.IMREAD_COLOR)
            undis_image = cv2.imdecode(np.frombuffer(data.undist_img.data, np.uint8), cv2.IMREAD_COLOR)

            # START: Image Processing
            # Convert to HSV color space
            hsv = cv2.cvtColor(undis_image, cv2.COLOR_BGR2HSV)
            # Define HSV range for blue (adjust if needed)
            lower_blue = np.array([100, 150, 50])
            upper_blue = np.array([140, 255, 255])

            # Threshold the HSV image to get only blue colors
            mask = cv2.inRange(hsv, lower_blue, upper_blue)

            # Morphological operations to clean noise
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            # Find contours in the mask
            _, contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Draw bounding boxes around detected objects
            for cnt in contours:
                if cv2.contourArea(cnt) > 300:  # Filter out small noise
                    x, y, w, h = cv2.boundingRect(cnt)
                    center_x = x + w // 2
                    center_y = y + h // 2
                    cv2.rectangle(undis_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    # Draw crosshair at center
                    cv2.line(undis_image, (center_x - 5, center_y), (center_x + 5, center_y), (0, 0, 255), 2)
                    cv2.line(undis_image, (center_x, center_y - 5), (center_x, center_y + 5), (0, 0, 255), 2)
                    rospy.loginfo("Blue pen object found at: ({}, {})".format(center_x, center_y))

            # Show results
            # END: Image Processing

            # Stack the images horizontally
            side_by_side = np.hstack((raw_image, undis_image))

            # Add a label to each half (optional)
            cv2.putText(side_by_side, "Original", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.putText(side_by_side, "Undistorted", (undis_image.shape[1] + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

            # Display the result
            cv2.imshow("Original vs. Undistorted", side_by_side)
            
            #cv2.waitKey(1)
            cv2.waitKey(1)  # Non-blocking update

            #rospy.loginfo("Trying to show camera")
            # Ensure the window updates instantly
            #cv2.imshow("Camera View", undis_image)
            #cv2.waitKey(1)  # Keep at 1 to prevent blocking
        except CvBridgeError as err:
            rospy.logerr("Error converting image: {}".format(err))
            return


    def cleanup(self):
        cv2.destroyAllWindows()

if __name__ == "__main__":
    # Initialize the node
    rospy.init_node('camera_viewer_node', anonymous=True)
    camera_node = CameraSubscriberNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down image viewer node.")
    finally:
        camera_node.cleanup()
