#!/usr/bin/env python3

import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
from motor_control.msg import motor_cmd
from motor_control.srv import MoveStraight, Rotate, Stop, ConstRotate, ConstStraight, DriveLeftwheel, DriveRightwheel


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

        self.cmd_rate = 2.0 # generate move commands at 2 Hz
        self.cmd_dt = 1.0 / self.cmd_rate
        self.last_cmd_ts = rospy.Time.now()
        self.latest_center = None  # updated every frame
        self.timer = rospy.Timer(rospy.Duration(self.cmd_dt), self.timer_cb)
        self.new_cmd = 0 
        self.distance_cmd = 0.08
        self.angle_cmd = 0.3
        self.turn_vel = 0.5
        self.move_vel = 0.2

        self.middle = 640 / 2
        
        rospy.on_shutdown(self.shutdown) # Shutdown hook to clean up resources

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
            # hsv = cv2.cvtColor(undis_image, cv2.COLOR_BGR2HSV)

            # v_channel = hsv[:,:,2]  # brightness

            # # Compute adaptive thresholds for V channel
            # mean_v = np.mean(v_channel)
            # std_v = np.std(v_channel)

            # lower_v = max(0, mean_v - 1.5 * std_v)
            # upper_v = min(255, mean_v + 1.5 * std_v)

            # dtype = hsv.dtype
            # # lower_white = np.array([0, 0, lower_v], dtype=dtype)
            # # upper_white = np.array([180, 40, upper_v], dtype=dtype)
            # lower_white = np.array([0, 0, 180])
            # upper_white = np.array([180, 70, 255])

            # # lower_white = np.array([0, 0, lower_v])
            # # upper_white = np.array([180, 40, upper_v])

            # mask = cv2.inRange(hsv, lower_white, upper_white)

            # # Morphological operations to clean noise
            # kernel = np.ones((5, 5), np.uint8)
            # mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            # mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            # segmented_image = cv2.bitwise_and(undis_image, undis_image, mask=mask)

            gray = cv2.cvtColor(undis_image, cv2.COLOR_BGR2GRAY)

            # Apply Gaussian Blur to smooth noise
            blurred = cv2.GaussianBlur(gray, (5,5), 0)

            # Canny Edge Detection
            edges = cv2.Canny(blurred, 50, 150)
           

            MIN_AREA_TRACK = 20  # Minimum area for track marks

            # get a list of contours
            contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

            lines = []

            for contour in contours:
                area = cv2.contourArea(contour)
                if area < 50:  # Skip tiny noise
                    continue

                x, y, w, h = cv2.boundingRect(contour)
                if h < 20:  # Skip very flat shapes
                    continue

                # Optional: Filter by angle
                [vx, vy, x0, y0] = cv2.fitLine(contour, cv2.DIST_L2, 0, 0.01, 0.01)
                angle = np.arctan2(vy, vx) * 180 / np.pi

                # Accept lines with angles between -70 and +70 degrees
                if -70 < angle < 70:
                    M = cv2.moments(contour)
                    if M['m00'] != 0:
                        cx = int(M["m10"]/M["m00"])
                        cy = int(M["m01"]/M["m00"])
                        lines.append({'x': cx, 'y': cy})
                        cv2.drawContours(undis_image, [contour], -1, (255,0,0), 2)

            if lines:
                # Take the line that is the most centered
                lines = sorted(lines, key=lambda l: abs(l['x'] - self.middle))
                
                cv2.circle(edges, (lines[0]['x'], lines[0]['y']), 5, (0, 255, 0), -1)
                cv2.circle(undis_image, (lines[0]['x'], lines[0]['y']), 5, (0, 255, 0), -1)
                self.latest_center = (lines[0]['x'], lines[0]['y'])
            else:
                self.latest_center = None

            #cv2.imshow("Segmented Image", segmented_image)
            cv2.imshow("edges", edges)
            cv2.imshow("Undistorted Image", undis_image)
            #cv2.imshow("Mask", mask)

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
            self.motor_cmd(0.5, 0, 1, 0)

            return

        center_x, center_y = self.latest_center

        error = center_x - self.middle
        
        # Lets determine the angle, negative means turn left, positive means turn right
        # angle is in radians
        angle = np.arctan2(error, 100)  # 100 is a scaling factor for the angle, 320 error is 1.26 radians == 72 degrees
        
        # scale speed based on the absolute error
        velocity = max(self.move_vel * (1 - min(abs(error) / self.middle, 1)), 0.5)  # Scale velocity based on error, min speed is 0.5
        # determine the 2 velocities for the left and right wheel
        velocity_right = velocity * (1 - (angle / np.pi)/2)
        velocity_left = velocity * (1 + (angle / np.pi)/2)

        self.call_left_wheel(1, velocity_left)
        self.call_right_wheel(1, velocity_right)

    # <================= Motor command function =================>
    def motor_cmd(self, velocity, distance, angle, blocking):
        # Create a new motor_cmd message
        new_motor_cmd = motor_cmd()
        self.new_cmd += 1
        new_motor_cmd.new_mesg = self.new_cmd
        new_motor_cmd.velocity = velocity
        new_motor_cmd.distance = distance
        new_motor_cmd.angle = angle
        new_motor_cmd.blocking = blocking

        # Publish the command
        self.pub_cmd.publish(new_motor_cmd)

    def call_left_wheel(self, direction: int, speed: float) -> None:
        rospy.wait_for_service('left_wheel_vel')
        try:
            proxy = rospy.ServiceProxy('left_wheel_vel', DriveLeftwheel)
            resp = proxy(direction, speed)
            print("Left wheel command success:", resp.success)
        except rospy.ServiceException as e:
            print("Service call failed:", e)


    def call_right_wheel(self, direction: int, speed: float) -> None:
        rospy.wait_for_service('right_wheel_vel')
        try:
            proxy = rospy.ServiceProxy('right_wheel_vel', DriveRightwheel)
            resp = proxy(direction, speed)
            print("Right wheel command success:", resp.success)
        except rospy.ServiceException as e:
            print("Service call failed:", e)

    def call_stop(self):
        rospy.wait_for_service('stop')
        try:
            proxy = rospy.ServiceProxy('stop', Stop)
            resp = proxy()
            print("Stop success:", resp.success)
        except rospy.ServiceException as e:
            print("Service call failed:", e)


    def shutdown(self):
        rospy.loginfo("Shutting down line follower node.")
        self.call_stop()        # Stop the robot
        cv2.destroyAllWindows() # Close all OpenCV windows

if __name__ == "__main__":
    # Initialize the node
    rospy.init_node('line_follower_node', anonymous=False, xmlrpc_port=45102, tcpros_port=45103)
    camera_node = CameraSubscriberNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down image viewer node.")
    