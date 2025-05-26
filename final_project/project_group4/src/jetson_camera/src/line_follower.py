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

            # Find contours in the mask
            _, contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # if there are no contours, set latest center to None
            if not contours:
                self.latest_center = None  # No blue object detected
                #return  # Exit early

            

            # Display the result
            cv2.imshow("Object tracking", undis_image)
            
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

        # Define thresholds for 3x3 grid regions in the image
        left_thresh = 214
        right_thresh = 426
        top_thresh = 160
        bottom_thresh = 320

        self.new_cmd += 1
        new_motor_cmd = motor_cmd()
        new_motor_cmd.new_mesg = self.new_cmd

        # # <================= Movement logic upper row =================>
        # if ((center_x < left_thresh) and (center_y < top_thresh)):
        #     rospy.loginfo("Blue pen is in the left upper corner.")
        #     # move forward + turn left
        #     # generate motor 
        #     self.motor_cmd(self.turn_vel, 0.0, -self.angle_cmd) # turn left
        #     #self.motor_cmd(0.5, self.distance_cmd/2, 0.0) # move forward

        # elif ((left_thresh <= center_x < right_thresh) and (center_y < top_thresh)):
        #     rospy.loginfo("Blue pen is in the middle up.")
        #     # move forward (i think)
        #     self.motor_cmd(self.move_vel, self.distance_cmd, 0.0) # move forward

        # elif ((center_x >= right_thresh) and (center_y < top_thresh)):
        #     rospy.loginfo("Blue pen is in the right upper corner.")
        #     # move forward + turn right
        #     self.motor_cmd(self.turn_vel, 0.0, self.angle_cmd) # turn right
        #     #self.motor_cmd(0.5, self.distance_cmd/2, 0.0) # move forward

        # # <================= Movement logic middle row =================>
        # elif ((center_x < left_thresh) and (top_thresh <= center_y < bottom_thresh)):
        #     rospy.loginfo("Blue pen is in the middle left.")
        #     # turn left
        #     self.motor_cmd(self.turn_vel, 0.0, -self.angle_cmd) # turn left

        # elif ((left_thresh <= center_x < right_thresh) and (top_thresh <= center_y < bottom_thresh)):
        #     rospy.loginfo("Blue pen is in the middle.")
        #     # do nothing
        #     self.motor_cmd(0.0, 0.0, 0.0) # turn left

        # elif ((center_x >= right_thresh) and (top_thresh <= center_y < bottom_thresh)):
        #     rospy.loginfo("Blue pen is in the middle right.")
        #     # turn right
        #     self.motor_cmd(self.turn_vel, 0.0, self.angle_cmd) # turn right

        # # <================= Movement logic lower row =================>
        # elif ((center_x < left_thresh) and (center_y >= bottom_thresh)):
        #     rospy.loginfo("Blue pen is in the left lower corner.")
        #     # move backward + turn left
        #     self.motor_cmd(self.turn_vel, 0.0, -self.angle_cmd) # turn left
        #     #self.motor_cmd(0.5, -self.distance_cmd/2, 0.0) # move backwards

        # elif ((left_thresh <= center_x < right_thresh) and (center_y >= bottom_thresh)):
        #     rospy.loginfo("Blue pen is in the middle down.")
        #     # move backward (i think)
        #     self.motor_cmd(self.move_vel, -self.distance_cmd/2, 0.0) # move backwards

        # elif ((center_x >= right_thresh) and (center_y >= bottom_thresh)):
        #     rospy.loginfo("Blue pen is in the right lower corner.")
        #     # move backward + turn right
        #     self.motor_cmd(self.turn_vel, 0.0, self.angle_cmd) # turn right
        #     #self.motor_cmd(0.5, -self.distance_cmd/2, 0.0) # move backwards
        # # this should never be possible
        # else:
        #     rospy.loginfo_throttle(2.0, "Pen in unknown position.")
        #     return

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
