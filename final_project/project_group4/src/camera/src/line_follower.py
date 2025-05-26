#!/usr/bin/env python2

import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
from motor_control.msg import motor_cmd


class CameraSubscriberNode:
    def __init__(self):
        self.initialized = False
        rospy.loginfo("Initializing camera object detection node...")
        self.bridge = CvBridge()
        
        # Construct subscriber
        self.sub_image = rospy.Subscriber(
            "/camera/image_proc",
            CompressedImage,
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

        self.cmd_rate = 10.0 # generate move commands at 5Hz
        self.cmd_dt = 1.0 / self.cmd_rate
        self.last_cmd_ts = rospy.Time.now()
        self.latest_center = None  # updated every frame
        self.timer = rospy.Timer(rospy.Duration(self.cmd_dt), self.timer_cb)
        self.new_cmd = 0 
        self.distance_cmd = 0.08
        self.angle_cmd = 0.3
        self.turn_vel = 0.5
        self.move_vel = 0.6

        self.middle = 640 / 2

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
            undis_image = cv2.imdecode(np.frombuffer(data.data, np.uint8), cv2.IMREAD_COLOR)

            # <================= START: Image Processing ======================>
            # Convert to HSV color space
            hsv = cv2.cvtColor(undis_image, cv2.COLOR_BGR2HSV)

            # Define HSV range for white (adjust if needed) THIS IS FOR DETECTING THE Line
            lower_white = np.array([0, 0, 200])  # Lower bound for white
            upper_white = np.array([180, 30, 255])  # Upper bound for white

            # Threshold the HSV image to get only blue colors IT MASK ALL OTHER COLORS EXCEPT WHITE (as defined above)
            mask = cv2.inRange(hsv, lower_white, upper_white)

            # Morphological operations to clean noise
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            segmented_image = cv2.bitwise_and(undis_image, undis_image, mask=mask)
           

            MIN_AREA_TRACK = 20  # Minimum area for track marks

            # get a list of contours
            _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

            lines = []

            for contour in contours:
                M = cv2.moments(contour)

                if (M['m00'] > MIN_AREA_TRACK):
                    # Contour is part of the track
                    cx = int(M["m10"]/M["m00"])
                    xy = int(M["m01"]/M["m00"])
                    lines.append({'x': cx, 'y': xy})

            if lines:
                cv2.circle(segmented_image, (lines[0]['x'], lines[0]['y']), 5, (0, 255, 0), -1)
                self.latest_center = (lines[0]['x'], lines[0]['y'])
            else:
                self.latest_center = None

            cv2.imshow("Segmented Image", segmented_image)
            cv2.waitKey(1)  # Non-blocking update

        except CvBridgeError as err:
            rospy.logerr("Error converting image: {}".format(err))
            return


# <=================== Motor control =========================>
    def timer_cb(self, event):
        if not self.initialized:
            return
        
        if self.latest_center is None:
            rospy.loginfo_throttle(2.0, "No line detected recently. No movement.")
            return

        center_x, center_y = self.latest_center

        error = center_x - self.middle
        
        # Lets determine the angle, negative means turn left, positive means turn right
        # angle is in radians
        angle = np.arctan2(error, 100)  # 100 is a scaling factor for the angle, 320 error is 1.26 radians == 72 degrees
        
        # scale speed based on the absolute error
        velocity = max(self.move_vel * (1 - min(abs(error) / self.middle, 1)), 0.5)  # Scale velocity based on error, min speed is 0.5


        self.new_cmd += 1
        new_motor_cmd = motor_cmd()
        new_motor_cmd.new_mesg = self.new_cmd

        new_motor_cmd.velocity = velocity
        new_motor_cmd.distance = self.distance_cmd
        new_motor_cmd.angle = angle
        new_motor_cmd.blocking = 0

    # <================= Motor command function =================>
    def motor_cmd(self, velocity, distance, angle):
        # Create a new motor_cmd message
        new_motor_cmd = motor_cmd()
        self.new_cmd += 1
        new_motor_cmd.new_mesg = self.new_cmd
        new_motor_cmd.velocity = velocity
        new_motor_cmd.distance = distance
        new_motor_cmd.angle = angle

        # Publish the command
        self.pub_cmd.publish(new_motor_cmd)


    def cleanup(self):
        cv2.destroyAllWindows()

if __name__ == "__main__":
    # Initialize the node
    rospy.init_node('line_follower_node', anonymous=False, xmlrpc_port=45102, tcpros_port=45103)
    camera_node = CameraSubscriberNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down image viewer node.")
    finally:
        camera_node.cleanup()
