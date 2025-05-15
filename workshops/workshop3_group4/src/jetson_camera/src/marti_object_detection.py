#!/usr/bin/env python2

import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
from jetson_camera.msg import twovids
from motor_control.msg import motor_cmd


class CameraSubscriberNode:
    def __init__(self):
        self.initialized = False
        rospy.loginfo("Initializing camera object detection node...")
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
        rospy.loginfo("Camera object detection node initialized!")


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
                    if (center_x <= 214 and center_y <= 160):
                        rospy.loginfo("Blue pen is in the left upper corner.")
                        # move forward + turn left
                    elif ((center_x > 214 or center_x < 426) and center_y <= 160):
                        rospy.loginfo("Blue pen is in the middle up.")
                        # move forward (i think)
                    elif (center_x > 426 and center_y <= 160):
                        rospy.loginfo("Blue pen is in the right upper corner.")
                        # move forward + turn right
                    elif (center_x <= 214 and (center_y > 160 or center_y < 320)):
                        rospy.loginfo("Blue pen is in the middle left.")
                        # turn left
                    elif ((center_x > 214 or center_x < 426) and (center_y > 160 or center_y < 320)):
                        rospy.loginfo("Blue pen is in the middle.")
                        # do nothing
                    elif (center_x >= 426 and (center_y > 160 or center_y < 320)):
                        rospy.loginfo("Blue pen is in the middle right.")
                        # turn right
                    elif (center_x <= 214 and center_y >= 320):
                        rospy.loginfo("Blue pen is in the left lower corner.")
                        # move backward + turn left
                    elif ((center_x > 214 or center_x < 426) and center_y >= 320):
                        rospy.loginfo("Blue pen is in the middle down.")
                        # move backward (i think)
                    elif (center_x >= 426 and center_y >= 320):
                        rospy.loginfo("Blue pen is in the right lower corner.")
                        # move backward + turn right
                    else:
                        rospy.loginfo("Blue pen is in the unknown position.")
            # END: Image Processing

            # SHOW THE RESULTS:
            # Stack the images horizontally
            side_by_side = np.hstack((raw_image, undis_image))

            # Add a label to each half (optional)
            cv2.putText(side_by_side, "Original", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.putText(side_by_side, "Undistorted", (undis_image.shape[1] + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

            # Display the result
            cv2.imshow("Original vs. Undistorted", side_by_side)
            
            cv2.waitKey(1)  # Non-blocking update

        except CvBridgeError as err:
            rospy.logerr("Error converting image: {}".format(err))
            return


    def cleanup(self):
        cv2.destroyAllWindows()

if __name__ == "__main__":
    # Initialize the node
    rospy.init_node('object_detection_node')
    camera_node = CameraSubscriberNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down image viewer node.")
    finally:
        camera_node.cleanup()
