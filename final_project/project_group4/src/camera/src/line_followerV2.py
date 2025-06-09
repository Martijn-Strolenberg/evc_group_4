#!/usr/bin/env python3

import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
import threading
from motor_control.srv import MoveStraight, Rotate, Stop, ConstRotate, ConstStraight, DriveLeftwheel, DriveRightwheel
from motor_control.srv import SetMaxSpeed, SetMaxSpeedResponse
from sensor_reading.srv import ButtonPressed, ButtonPressedResponse
from sensor_reading.srv import CollisionDetection, CollisionDetectionResponse
from std_msgs.msg import Int8
from camera.msg import ObjectDetection



class CameraSubscriberNode:
    def __init__(self):
        self.initialized = False
        rospy.loginfo("Initializing camera object detection node...")
        self.bridge = CvBridge()
        
        # Construct subscriber to receive compressed images from the camera
        self.sub_image = rospy.Subscriber(
            "/camera/image_proc",
            CompressedImage,
            self.image_cb,
            buff_size=2**24,
            queue_size=1
        )

        # Construct subscriber to receive button state updates
        self.sub_button = rospy.Subscriber(
            "/button_state",
            Int8,
            self.button_cb,
            buff_size=2**24,
            queue_size=10
        )

        # Construct subscriber to receive object detection results
        self.sub_object_detected = rospy.Subscriber(
            "/object_detection",
            ObjectDetection,
            self.object_detected_cb,
            buff_size=2**24,
            queue_size=10
        )


        self.cmd_rate = 10.0 # generate move commands at 10 Hz
        self.cmd_dt = 1.0 / self.cmd_rate
        self.last_cmd_ts = rospy.Time.now()
        self.latest_center = None  # updated every frame
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

        self.robot_enabled = False # Flag to enable/disable robot movement

        self.middle = 640 / 2

        self.kp = rospy.get_param("~kp", 3.2)          # controller gain default=3.2
        self.kp_exp = rospy.get_param("~kp_exp", 4.0)  # Exponent for error scaling, default is 4.0
        rospy.Timer(rospy.Duration(0.5), self.param_watchdog_cb)  # Check every 0.5s
        
        rospy.on_shutdown(self.shutdown) # Shutdown hook to clean up resources

        # Initialize motor command services API
        #services = MotorServices()
        rospy.Service('set_max_speed', SetMaxSpeed, self.set_max_speed) # service to receive the maximum speed
        rospy.Service('button_pressed', ButtonPressed, self.rec_button_pressed) # service to receive if the button is pressed
        rospy.Service('collision_detection', CollisionDetection, self.rec_collision_detection) # service to receive if a collison is almost detected

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

    def button_cb(self, data):
        if not self.initialized:
            return

        if data.data == 1:
            rospy.loginfo("Button pressed once!")
            if not self.robot_enabled:
                rospy.loginfo("Enabling robot movement.")
                self.robot_enabled = True  # Enable robot movement
            else:
                rospy.loginfo("Disabling robot movement.")
                self.robot_enabled = False
                self.call_stop()  # Stop the robot if button is pressed again
    
    def object_detected_cb(self, data):
        if not self.initialized:
            return

        if data.object_type == 1:
            rospy.loginfo("Blue object detected: at (%d,%d)", data.x_coordinate, data.y_coordinate)
        elif data.object_type == 2:
            rospy.loginfo("Orange object detected: at (%d,%d)", data.x_coordinate, data.y_coordinate)
        elif data.object_type == 3:
            rospy.loginfo("Green object detected: at (%d,%d)", data.x_coordinate, data.y_coordinate)
        elif data.object_type == 4:
            rospy.loginfo("Red object detected: at (%d,%d)", data.x_coordinate, data.y_coordinate)
            # Here you can add logic to handle the detected object, e.g., stop the robot or change behavior
            # For now, we just log it
        else:
            rospy.loginfo("No object detected.")
            


# <============== Image Processing =================>
    def image_cb(self, data):
        if not self.initialized:
            return

        try:

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
            best_centroid = None
            best_area = 0

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


                if area > best_area and area < 10000:  # Only consider contours with area < 10000
                    best_area = area
                    best_centroid = (cx, cy)

            self.latest_center = best_centroid  # Save the result for control logic

            if best_centroid:
                # Draw the selected centroid on the original image
                cv2.circle(undis_image, best_centroid, 5, (0, 255, 0), -1)
                cv2.putText(undis_image, f"Centroid: {best_centroid}", (best_centroid[0] + 10, best_centroid[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                #add the area of the contour
                cv2.putText(undis_image, f"Area: {best_area}", (best_centroid[0] + 10, best_centroid[1] + 10),
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

        if self.robot_enabled: # Only execute if the robot is enabled

            if self.latest_center is None:
                rospy.loginfo_throttle(2.0, "No line detected recently. No movement.")
                return

            center_x, center_y = self.latest_center

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


    # ================ Motor command services ======================== #
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

    # ============== Button pressed service =================== #
    def rec_button_pressed(self, req):
        if req == 1:
            rospy.loginfo("Button pressed status=%d, on", req.pressed)
            return ButtonPressedResponse(True)
        if req == 0:
            rospy.loginfo("Button pressed status=%d, off", req.pressed)
            return ButtonPressedResponse(True)
        else:
            rospy.logwarn("Button pressed status=%d, unknown", req.pressed)
            # If the button pressed status is not 1 or 0, return False
        return ButtonPressedResponse(False)

    # ============== Collison detection service ============== #
    def rec_collision_detection(self, req):
        if req.collision == True:
            rospy.logwarn("Collision detected!")
            return CollisionDetectionResponse(True)
        if req.collision == False:
            rospy.logwarn("Collision danger is over!")
            return CollisionDetectionResponse(True)
        return CollisionDetectionResponse(False)

    # ----------------------------OCR-speed-set-----------------
    def set_max_speed(self, req):
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
    