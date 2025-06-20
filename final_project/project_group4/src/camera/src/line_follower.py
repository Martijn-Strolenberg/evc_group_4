#!/usr/bin/env python3

import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
import threading
from motor_control.msg import motor_cmd
from motor_control.srv import MoveStraight, Rotate, Stop, ConstRotate, ConstStraight, DriveLeftwheel, DriveRightwheel
from motor_control.srv import SetMaxSpeed, SetMaxSpeedResponse




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

        self.cmd_rate = 10.0 # generate move commands at 10 Hz
        self.cmd_dt = 1.0 / self.cmd_rate
        self.last_cmd_ts = rospy.Time.now()
        self.timer = rospy.Timer(rospy.Duration(self.cmd_dt), self.timer_cb)
        self.new_cmd = 0 
        self.distance_cmd = 0.08
        self.angle_cmd = 0.3
        self.turn_vel = 0.5
        self.move_vel = 0.1
        self.max_speed = 0.7  # Maximum speed for the wheels, can be set via service call
        self.vel_r = 0.0
        self.vel_l = 0.0
        self.er = 0.0  # error for debugging purposes
        self.min_area = 1000
        self.max_area = 10000  # Maximum area for the contour to be considered valid
        self.best_centroid = None

        self.middle = 640 / 2

        self.kp = rospy.get_param("~kp", 2.0)
        self.kp_exp = rospy.get_param("~kp_exp", 2.0)  # Exponent for error scaling, default is 1.0
        rospy.Timer(rospy.Duration(0.5), self.param_watchdog_cb)  # Check every 0.5s
        
        rospy.on_shutdown(self.shutdown) # Shutdown hook to clean up resources

        # Initialize motor command services API
        #services = MotorServices()
        rospy.Service('set_max_speed', SetMaxSpeed, self.set_max_speed)

        self.initialized = True
        rospy.loginfo("Camera object detection node initialized!")

    def param_watchdog_cb(self, event):
        new_kp = rospy.get_param("~kp", self.kp)
        new_kp_exp = rospy.get_param("~kp_exp", self.kp_exp)
        new_max_speed = rospy.get_param("~max_speed", self.max_speed)
        new_move_vel = rospy.get_param("~move_vel", self.move_vel)
        if new_kp != self.kp:
            rospy.loginfo("kp changed from %.2f to %.2f", self.kp, new_kp)
            self.kp = new_kp
        if new_kp_exp != self.kp_exp:
            rospy.loginfo("kp_exp changed from %.2f to %.2f", self.kp_exp, new_kp_exp)
            self.kp_exp = new_kp_exp
        if new_max_speed != self.max_speed:
            rospy.loginfo("max_speed changed from %.2f to %.2f", self.max_speed, new_max_speed)
            self.max_speed = new_max_speed
        if new_move_vel != self.move_vel:
            rospy.loginfo("move_vel changed from %.2f to %.2f", self.move_vel, new_move_vel)
            self.move_vel = new_move_vel


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

            v_channel = hsv[:,:,2]  # brightness

            # Compute adaptive thresholds for V channel
            mean_v = np.mean(v_channel)
            std_v = np.std(v_channel)

            lower_v = max(0, mean_v - 1.5 * std_v)
            upper_v = min(255, mean_v + 1.5 * std_v)

            dtype = hsv.dtype
            # lower_white = np.array([0, 0, lower_v], dtype=dtype)
            # upper_white = np.array([180, 40, upper_v], dtype=dtype)
            lower_white = np.array([0, 0, 180])
            upper_white = np.array([180, 70, 255])

            # lower_white = np.array([0, 0, lower_v])
            # upper_white = np.array([180, 40, upper_v])

            mask = cv2.inRange(hsv, lower_white, upper_white)

            # Morphological operations to clean noise
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            segmented_image = cv2.bitwise_and(undis_image, undis_image, mask=mask)

            # Find contours in the mask but only the bottom 2/3 of the screen
            height, width = mask.shape[:2]
            y23 = int(height * 2 / 3)  # Only consider the bottom 2/3 of the image
            mask[0:y23, :] = 0  # Set the top part of the mask to zero
            
            y13 = int(height * 1 / 3)  # Only consider the bottom 1/3 of the image


            # #also set the bottom part of the mask to zero
            # mask[y13:height, :] = 0  # Set the bottom part of the mask to zero



            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            best_area = 0
            centroids = []

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < 100:
                    continue
                M = cv2.moments(cnt)
                if M["m00"] == 0:
                    continue
                # Find topmost point (lowest y) of the contour
                topmost = tuple(cnt[cnt[:, :, 1].argmin()][0])
                cx, cy = topmost


                # top x of this contour


                if area > self.min_area and area < self.max_area:  # Only consider contours with area < 10000
                    centroids.append((cx, cy))

            #best_centroid is the centroid closest to the middle of the image if uninitialized
            # otherwise it is the centroid closest to the last detected centroid
            if self.best_centroid is not None:
                if centroids:
                    self.best_centroid = min(centroids, key=lambda c: abs((c[0] - self.best_centroid[0])**2 + (c[1] - self.best_centroid[1])**2))
                    best_area = cv2.contourArea(cv2.convexHull(np.array([self.best_centroid])))
            else:
                if centroids:
                    self.best_centroid = min(centroids, key=lambda c: abs((c[0] - self.middle)**2 + (c[1] - y23)**2))
                    best_area = cv2.contourArea(cv2.convexHull(np.array([self.best_centroid])))
                else:
                    self.best_centroid = None
                    best_area = 0
            

            if self.best_centroid:
                # Draw the selected centroid on the original image
                cv2.circle(undis_image, self.best_centroid, 5, (0, 255, 0), -1)
                cv2.putText(undis_image, f"Centroid: {self.best_centroid}", (self.best_centroid[0] + 10, self.best_centroid[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                #add the area of the contour
                cv2.putText(undis_image, f"Area: {best_area}", (self.best_centroid[0] + 10, self.best_centroid[1] + 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            #print the error and velocities at the top left

            cv2.putText(undis_image, f"Error: {self.er:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.putText(undis_image, f"Velocity Left: {self.vel_l:.2f}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.putText(undis_image, f"Velocity Right: {self.vel_r:.2f}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.putText(undis_image, f"Max Speed: {self.max_speed:.2f}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.putText(undis_image, f"kp: {self.kp:.2f}", (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.putText(undis_image, f"kp_exp: {self.kp_exp:.2f}", (10, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.putText(undis_image, f"base velocity: {self.move_vel}", (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            # #make an image with all the contours that are large enough
            # contours_image = np.zeros_like(undis_image)
            # for cnt in contours:
            #     area = cv2.contourArea(cnt)
            #     if area < 100:
            #         continue
            #     cv2.drawContours(contours_image, [cnt], -1, (255, 255, 255), 1)
            #     # add area
            #     cv2.putText(contours_image, f"Area: {area}", (int(M["m10"] / M["m00"]) + 10, int(M["m01"] / M["m00"]) - 10),
            #                 cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            cv2.imshow("Undistorted Image", undis_image)
            #cv2.imshow("Mask", mask)
            # cv2.imshow("Counters Image", contours_image)

            cv2.waitKey(1)  # Non-blocking update

        except CvBridgeError as err:
            rospy.logerr("Error converting image: {}".format(err))
            return


# <=================== Motor control =========================>
    def timer_cb(self, event):
        if not self.initialized:
            return
        
        if self.best_centroid is None:
            rospy.loginfo_throttle(2.0, "No line detected recently. No movement.")
            self.motor_cmd(0.5, 0, 1, 0)

            return

        center_x, center_y = self.best_centroid

        error = (center_x - self.middle) * self.kp/self.middle
        if error < 0:
            sign = -1
        else:
            sign = 1
        error = (abs(error) ** self.kp_exp) + 1.0  # Always ≥ 1
        
        # Lets determine the angle, negative means turn left, positive means turn right
        # angle is in radians
        # angle = np.arctan2(error, 100)  # 100 is a scaling factor for the angle, 320 error is 1.26 radians == 72 degrees
        
        # scale speed based on the absolute error
        #velocity = max(self.move_vel * (1 - min(abs(error) / self.middle, 1)), 0.3)  # Scale velocity based on error, min speed is 0.5
        # determine the 2 velocities for the left and right wheel

        # Default directions
        dir_left = 1
        dir_right = 1

        velocity_right = self.move_vel
        velocity_left = self.move_vel

        # If error is to the left
        if sign == -1:
            velocity_right = self.move_vel * error
            velocity_left = self.move_vel / error
        else:
            velocity_left = self.move_vel * error
            velocity_right = self.move_vel / error

        # Limit to max speed
        velocity_left = min(velocity_left, self.max_speed)
        velocity_right = min(velocity_right, self.max_speed)

        self.vel_r = velocity_right
        self.vel_l = velocity_left
        self.er = error

        # Send motor commands
        self.call_left_wheel(dir_left, velocity_left)
        self.call_right_wheel(dir_right, velocity_right)

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
    # ----------------------------OCR-speed-set-----------------
    def set_max_speed(self,req):
        if req.speed <= 0:
            rospy.logwarn("Speed must be >0")
            return SetMaxSpeedResponse(False)
        # Get the current status of the robot and set the goal parameters

        if req.speed > 0: # Check if distance is positive (drive forwards)
            # Move straight forward command

            self.move_vel = req.speed
            self.turn_vel = self.move_vel - 0.1
            rospy.loginfo("Chaning max speed to %.2fm/s", req.speed)
            return SetMaxSpeedResponse(True)

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
    finally:
        camera_node.shutdown()
    