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
from std_msgs.msg import Int8, Bool, UInt8
from camera.msg import ObjectDetection



class CameraSubscriberNode:
    def __init__(self):
        self.initialized = False
        rospy.loginfo("Initializing camera object detection node...")
        self.bridge = CvBridge()
        
        # subscriber to receive compressed images from the camera
        self.sub_image = rospy.Subscriber(
            "/camera/image_proc",
            CompressedImage,
            self.image_cb,
            buff_size=2**24,
            queue_size=1
        )

        # subscriber to receive button state updates
        self.sub_button = rospy.Subscriber(
            "/button_state",
            Int8,
            self.button_cb,
            buff_size=2**24,
            queue_size=10
        )

        # subscriber to receive object detection results
        self.sub_object_detected = rospy.Subscriber(
            "/object_detection",
            ObjectDetection,
            self.object_detected_cb,
            buff_size=2**24,
            queue_size=10
        )
        # subscriber to receive qr readings
        self.sub_qr_detected = rospy.Subscriber(
            "/find_object",
            UInt8,
            self.qr_detected_cb,
            buff_size=2**24,
            queue_size=10            
        )

        # subscriber to receive collision detection status
        self.sub_collision_detection = rospy.Subscriber(
            "/collision_detection",
            Bool,
            self.collision_detected_cb,
            buff_size=2**24,
            queue_size=10
        )

        # Object Detection
        self.find_object = -1
        self.detected_object = 0

        # For robot and vision
        self.robot_enabled = False
        self.cmd_rate = 20.0 # to generate move commands 
        self.cmd_dt = 1.0 / self.cmd_rate
        self.last_cmd_ts = rospy.Time.now()
        self.latest_center = None  # updated every frame
        self.timer = rospy.Timer(rospy.Duration(self.cmd_dt), self.timer_cb)
        self.middle = 640 / 2

        # Movement variables
        self.turn_vel = 0.55
        self.move_vel = 0.2 # BASE SPEED OF THE ROBOT
        self.max_speed = 0.55  # Maximum speed for the wheels, can be set via service call
        self.vel_r = 0.0
        self.vel_l = 0.0
        self.er = 0.0  # error for debugging purposes

        self.robot_enabled = False # Flag to enable/disable robot movement

        # Parameters for the controller
        self.kp = rospy.get_param("~kp", 3.0)          # controller gain default=3.2
        self.kp_exp = rospy.get_param("~kp_exp", 1.5)  # Exponent for error scaling, default is 4.0

        rospy.on_shutdown(self.shutdown) # Shutdown hook to clean up resources

        # Initialize motor command services API
        rospy.Service('set_max_speed', SetMaxSpeed, self.set_max_speed) # service to receive the maximum speed
        rospy.Service('button_pressed', ButtonPressed, self.rec_button_pressed) # service to receive if the button is pressed
        rospy.Service('collision_detection', CollisionDetection, self.rec_collision_detection) # service to receive if a collison is almost detected

        self.initialized = True
        rospy.loginfo("Camera object detection node initialized!")


    def button_cb(self, data): # button so that line follower can start
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

    def collision_detected_cb(self, data):
        if not self.initialized: # Check if the node is initialized
            return 

        if data.data == True:
            rospy.loginfo("Collsion Detected!")
            if self.robot_enabled:  # Only stop if the robot is enabled
                self.robot_enabled = False  # Disable robot movement
                rospy.loginfo("Stopping robot due to collision.")
                
        elif data.data == False:
            rospy.loginfo("No collsion anymore...")
            if not self.robot_enabled:     # Only re-enable if the robot was stopped
                self.robot_enabled = True  # Re-enable robot movement
                rospy.loginfo("Re-enabling robot movement after collision cleared.")
    
    def object_detected_cb(self, data): # Detect if there is a specific coloured object
        if not self.initialized:
            return
        self.detected_object = data.object_type
        if data.object_type == 1:
            #rospy.loginfo("Blue object detected: at (%d,%d)", data.x_coordinate, data.y_coordinate)
            rospy.loginfo_throttle(2.0,"Blue object detected: at (%d,%d)", data.x_coordinate, data.y_coordinate)
        elif data.object_type == 2:
            #rospy.loginfo("Orange object detected: at (%d,%d)", data.x_coordinate, data.y_coordinate)
            rospy.loginfo_throttle(2.0,"Orange object detected: at (%d,%d)", data.x_coordinate, data.y_coordinate)
        elif data.object_type == 3:
            #rospy.loginfo("Green object detected: at (%d,%d)", data.x_coordinate, data.y_coordinate)
            rospy.loginfo_throttle(2.0,"Green object detected: at (%d,%d)", data.x_coordinate, data.y_coordinate)
        elif data.object_type == 4:
            #rospy.loginfo("Red object detected: at (%d,%d)", data.x_coordinate, data.y_coordinate)
            rospy.loginfo_throttle(2.0,"Red object detected: at (%d,%d)", data.x_coordinate, data.y_coordinate)
        else:
            rospy.loginfo("No object detected.")
            
    def qr_detected_cb(self,data): # Check if a qr code has been detected
        if not self.initialized:
            return
        self.find_object = data.data
        if data.data == 1:
            rospy.loginfo("Finding Blue object ")
        elif data.data == 2:
            rospy.loginfo("Finding Orange object ")
        elif data.data == 3:
            rospy.loginfo("Finding Green object")
        elif data.data == 4:
            rospy.loginfo("Finding Red object")        

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

            dtype = hsv.dtype

            # Applying mask
            lower_black = np.array([0, 0, 0])
            upper_black = np.array([180, 255, 75])

            mask = cv2.inRange(hsv, lower_black, upper_black)

            # Morphological operations to clean noise
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            segmented_image = cv2.bitwise_and(undis_image, undis_image, mask=mask)

            # Find contours in the mask but only the bottom 2/3 of the screen
            height, width = mask.shape[:2]
            y23 = int(height * 2 / 3)  # Only consider the bottom 1/3 of the image
            #y12 = int(height * 1 / 2)  # Only consider the bottom 1/2 of the image
            mask[0:y23, :] = 0  # Set the top part of the mask to zero


            _,contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
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


                if area > best_area and area < 10000:  # Only consider contours with area < 10000
                    best_area = area
                    best_centroid = (cx, cy)

            self.latest_center = best_centroid  # Save the result for control logic

            if best_centroid:
                # Draw the selected centroid on the original image
                cv2.circle(undis_image, best_centroid, 5, (0, 255, 0), -1)
                cv2.putText(undis_image, "Centroid: {thing1}".format(thing1 = best_centroid), (best_centroid[0] + 10, best_centroid[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                #add the area of the contour
                cv2.putText(undis_image, "Area: {thing2}".format(thing2 = best_area), (best_centroid[0] + 10, best_centroid[1] + 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            #print the error and velocities at the top left

            #cv2.putText(undis_image, f"Error: {self.er:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            #cv2.putText(undis_image, f"Velocity Left: {self.vel_l:.2f}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            #cv2.putText(undis_image, f"Velocity Right: {self.vel_r:.2f}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            #cv2.putText(undis_image, f"Max Speed: {self.max_speed:.2f}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            #cv2.putText(undis_image, f"kp: {self.kp:.2f}", (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            #cv2.putText(undis_image, f"kp_exp: {self.kp_exp:.2f}", (10, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            #cv2.putText(undis_image, f"base velocity: {self.move_vel}", (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)


            cv2.imshow("Undistorted Image", undis_image)
            cv2.imshow("Mask", mask)
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

            # if target object is found
            if self.find_object == self.detected_object:
                self.robot_enabled = False
                rospy.loginfo("Expected object has been detected. Stopping movement")
                self.call_stop()
                return

            if self.latest_center is None:
                rospy.loginfo("No line detected recently. No movement.")
                self.call_stop()
                self.robot_enabled = False
                return

            center_x, center_y = self.latest_center

            error = (center_x - self.middle) * self.kp/self.middle
            if error < 0:
                sign = -1
            else:
                sign = 1
            error = (abs(error) ** self.kp_exp) + 1.0  # Always >=1

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

            
            if velocity_left == self.max_speed:
                velocity_right = -velocity_right

            if velocity_right == self.max_speed:
                velocity_left = -velocity_left

            # Send motor commands
            self.call_left_wheel(dir_left, velocity_left)
            self.call_right_wheel(dir_right, velocity_right)


    # ================ Motor command services ======================== #
    def call_left_wheel(self, direction, speed): 
        rospy.wait_for_service('left_wheel_vel')
        try:
            proxy = rospy.ServiceProxy('left_wheel_vel', DriveLeftwheel)
            resp = proxy(direction, speed)
            
        except rospy.ServiceException as e:
            print("Service call failed:", e)

    def call_right_wheel(self, direction, speed): 
        rospy.wait_for_service('right_wheel_vel')
        try:
            proxy = rospy.ServiceProxy('right_wheel_vel', DriveRightwheel)
            resp = proxy(direction, speed)
            
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
    rospy.init_node('line_follower_node', anonymous=False, xmlrpc_port=45104, tcpros_port=45105)
    camera_node = CameraSubscriberNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down image viewer node.")
    finally:
        camera_node.shutdown()
    