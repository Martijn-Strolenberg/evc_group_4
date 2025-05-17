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

        # Publisher for motor commands (example)
        self.pub_cmd = rospy.Publisher(
            "/motor_control",
            motor_cmd,
            queue_size=10
        )

        self.cmd_rate = 2.0 # generate move commands at 5Hz
        self.cmd_dt = 1.0 / self.cmd_rate
        self.last_cmd_ts = rospy.Time.now()
        self.latest_center = None  # updated every frame
        self.timer = rospy.Timer(rospy.Duration(self.cmd_dt), self.timer_cb)
        self.new_cmd = 0 
        self.distance_cmd = 0.08
        self.angle_cmd = 0.3

        self.initialized = True
        rospy.loginfo("Camera object detection node initialized!")

# <============== Image Processing =================>
    def image_cb(self, data):
        if not self.initialized:
            return

        try:
            # Decode image without CvBridge
            #raw_image = cv2.imdecode(np.frombuffer(data.raw_img.data, np.uint8), cv2.IMREAD_COLOR)

            # only decode the undistorted image
            undis_image = cv2.imdecode(np.frombuffer(data.undist_img.data, np.uint8), cv2.IMREAD_COLOR)

            # START: Image Processing
            # Convert to HSV color space
            hsv = cv2.cvtColor(undis_image, cv2.COLOR_BGR2HSV)

            # Define HSV range for blue (adjust if needed) THIS IS FOR DETECTING THE BLUE PEN
            lower_blue = np.array([100, 150, 50])
            upper_blue = np.array([140, 255, 255])

            # Threshold the HSV image to get only blue colors IT MASK ALL OTHER COLORS EXCEPT BLUE (as defined above)
            mask = cv2.inRange(hsv, lower_blue, upper_blue)

            # Morphological operations to clean noise
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            # Find contours in the mask
            _, contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # if there are no contours, set latest center to None
            if not contours:
                self.latest_center = None  # No blue object detected
                #return  # Exit early

            # Draw bounding boxes around detected objects
            for cnt in contours:
                if cv2.contourArea(cnt) > 300:  # Filter out small noise (adjust as needed)
                    x, y, w, h = cv2.boundingRect(cnt)  # Get bounding box coordinates
                    center_x = x + w // 2               # Calculate center x coordinate of detected object
                    center_y = y + h // 2               # Calculate center y coordinate of detected object

                    # Object is detected and we update the latest center
                    self.latest_center = (center_x, center_y)  # Store latest pen position
                    cv2.rectangle(undis_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    # Draw crosshair at center
                    cv2.line(undis_image, (center_x - 5, center_y), (center_x + 5, center_y), (0, 0, 255), 2)
                    cv2.line(undis_image, (center_x, center_y - 5), (center_x, center_y + 5), (0, 0, 255), 2)
                    #rospy.loginfo("Blue pen object found at: ({}, {})".format(center_x, center_y))
                else:
                    # If no object is detected, set latest center to None
                    #rospy.loginfo("No object detected!.")
                    self.latest_center = None
            # END: Image Processing

            # SHOW THE RESULTS:
            # Stack the images horizontally
            # side_by_side = np.hstack((raw_image, undis_image))

            # Add a label to each half (optional)
            cv2.putText(undis_image, "Object Tracking", (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
            # cv2.putText(side_by_side, "Undistorted", (undis_image.shape[1] + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

            # Display the result
            cv2.imshow("Object tracking", undis_image)
            
            cv2.waitKey(1)  # Non-blocking update

        except CvBridgeError as err:
            rospy.logerr("Error converting image: {}".format(err))
            return


# <============== Motor control =================>
    def timer_cb(self, event):
        if not self.initialized:
            return
        
        if self.latest_center is None:
            rospy.loginfo_throttle(2.0, "No pen detected recently. No movement.")
            return

        center_x, center_y = self.latest_center

        # Define thresholds for 3x3 grid regions in the image
        left_thresh = 214
        right_thresh = 426
        top_thresh = 160
        bottom_thresh = 320

        self.new_cmd += 1
        new_motor_cmd = motor_cmd()
        new_motor_cmd.new_mesg = self.new_cmd

        # <================= Movement logic upper row =================>
        if ((center_x < left_thresh) and (center_y < top_thresh)):
            rospy.loginfo("Blue pen is in the left upper corner.")
            # move forward + turn left
            # generate motor command
            new_motor_cmd.velocity = 0.3
            new_motor_cmd.distance = 0.0
            new_motor_cmd.angle = -self.angle_cmd

        elif ((left_thresh <= center_x < right_thresh) and (center_y < top_thresh)):
            rospy.loginfo("Blue pen is in the middle up.")
            # move forward (i think)
            new_motor_cmd.velocity = 0.5
            new_motor_cmd.distance = self.distance_cmd
            new_motor_cmd.angle = 0.0

        elif ((center_x >= right_thresh) and (center_y < top_thresh)):
            rospy.loginfo("Blue pen is in the right upper corner.")
            # move forward + turn right
            new_motor_cmd.velocity = 0.3
            new_motor_cmd.distance = 0.0
            new_motor_cmd.angle = self.angle_cmd

        # <================= Movement logic middle row =================>
        elif ((center_x < left_thresh) and (top_thresh <= center_y < bottom_thresh)):
            rospy.loginfo("Blue pen is in the middle left.")
            # turn left
            new_motor_cmd.velocity = 0.3
            new_motor_cmd.distance = 0.0
            new_motor_cmd.angle = -self.angle_cmd

        elif ((left_thresh <= center_x < right_thresh) and (top_thresh <= center_y < bottom_thresh)):
            rospy.loginfo("Blue pen is in the middle.")
            # do nothing
            new_motor_cmd.velocity = 0.0
            new_motor_cmd.distance = 0.0
            new_motor_cmd.angle = 0.0

        elif ((center_x >= right_thresh) and (top_thresh <= center_y < bottom_thresh)):
            rospy.loginfo("Blue pen is in the middle right.")
            # turn right
            new_motor_cmd.velocity = 0.3
            new_motor_cmd.distance = 0.0
            new_motor_cmd.angle = self.angle_cmd

        # <================= Movement logic lower row =================>
        elif ((center_x < left_thresh) and (center_y >= bottom_thresh)):
            rospy.loginfo("Blue pen is in the left lower corner.")
            # move backward + turn left
            new_motor_cmd.velocity = 0.3
            new_motor_cmd.distance = 0.0
            new_motor_cmd.angle = -self.angle_cmd

        elif ((left_thresh <= center_x < right_thresh) and (center_y >= bottom_thresh)):
            rospy.loginfo("Blue pen is in the middle down.")
            # move backward (i think)
            new_motor_cmd.velocity = 0.5
            new_motor_cmd.distance = -self.distance_cmd
            new_motor_cmd.angle = 0.0

        elif ((center_x >= right_thresh) and (center_y >= bottom_thresh)):
            rospy.loginfo("Blue pen is in the right lower corner.")
            # move backward + turn right
            new_motor_cmd.velocity = 0.3
            new_motor_cmd.distance = 0.0
            new_motor_cmd.angle = self.angle_cmd

        else:
            rospy.loginfo_throttle(2.0, "Pen in unknown position.")
            return

        self.pub_cmd.publish(new_motor_cmd)


    def cleanup(self):
        cv2.destroyAllWindows()

if __name__ == "__main__":
    # Initialize the node
    rospy.init_node('object_detection_node', anonymous=False, xmlrpc_port=45102, tcpros_port=45103)
    camera_node = CameraSubscriberNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down image viewer node.")
    finally:
        camera_node.cleanup()
