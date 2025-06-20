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
from std_msgs.msg import Int8, Bool, UInt8, Float64
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

        # Construct subscriber to receive button state updates
        self.sub_speed_limit = rospy.Subscriber(
            "/max_speed",
            Float64,
            self.speed_limit_cb,
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
        # Construct subscriber to receive qr readings
        self.sub_qr_detected = rospy.Subscriber(
            "/find_object",
            UInt8,
            self.qr_detected_cb,
            buff_size=2**24,
            queue_size=10            
        )

        # Construct subscriber to receive collision detection status
        self.sub_collision_detection = rospy.Subscriber(
            "/collision_detection",
            Bool,
            self.collision_detected_cb,
            buff_size=2**24,
            queue_size=10
        )

        # Construct subscriber to receive which object to find
        self.sub_find_object = rospy.Subscriber(
            "/find_object",
            UInt8,
            self.find_object_cb,
            buff_size=2**24,
            queue_size=10
        )


        # Object Detection
        self.find_object = -1
        self.detected_object = 0


        self.cmd_rate = 30.0 # generate move commands at 10 Hz
        self.cmd_dt = 1.0 / self.cmd_rate
        self.last_cmd_ts = rospy.Time.now()
        self.latest_center = None  # updated every frame
        self.timer = rospy.Timer(rospy.Duration(self.cmd_dt), self.timer_cb)
        self.new_cmd = 0 
        self.distance_cmd = 0.08
        self.angle_cmd = 0.3
        self.turn_vel = 0.5

        self.move_vel = 0.18 # BASE SPEED OF THE ROBOT
        
        self.max_speed = 0.55  # Maximum speed for the wheels, can be set via service call
        self.vel_r = 0.0
        self.vel_l = 0.0
        self.er = 0.0  # error for debugging purposes

        self.record_video = False  # Set to False if you want to disable recording
        self.video_writer = None
        self.video_filename = f"video.avi"
        self.video_fps = 10
        self.video_size = (640, 480)  # Match the size of your input image
        if self.record_video:
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            self.video_writer = cv2.VideoWriter(self.video_filename, fourcc, self.video_fps, self.video_size)
            rospy.loginfo(f"Recording undistorted video to: {self.video_filename}")

        self.robot_enabled = True #False: if we want it to start with button press first # Flag to enable/disable robot movement
        self.best_centroid = None
        self.min_area = 1000  # Minimum area of the contour to consider it valid
        self.max_area = 20000  # Maximum area of the contour to consider it valid
        self.middle = 640 / 2

        self.centroids = []


        # Parameters for the controller ---------------------------------------------------------------------------------------
        self.kp = rospy.get_param("/kp", 6.5)          # controller gain default=3.2
        self.kp_exp = rospy.get_param("/kp_exp", 2.5)  # Exponent for error scaling, default is 4.0
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
        new_kp = rospy.get_param("/kp", self.kp)
        new_kp_exp = rospy.get_param("/kp_exp", self.kp_exp)
        new_max_speed = rospy.get_param("/max_speed", self.max_speed)
        new_move_vel = rospy.get_param("/move_vel", self.move_vel)
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

    def find_object_cb(self, data):
        if not self.initialized:
            return

        if data.data == 1:
            rospy.loginfo("Finding blue object...")
            # Here you can add logic to start searching for the blue object
            # For now, we just log it

    def speed_limit_cb(self, data):
        if not self.initialized:
            return

        if data.data == 0.3:
            rospy.logwarn("speed limit set to 0.3 m/s")
            self.move_vel = 0.10
        elif data.data == 0.4:
            rospy.logwarn("speed limit set to 0.4 m/s")
            self.move_vel = 0.13
        elif data.data == 0.5:
            rospy.logwarn("speed limit set to 0.5 m/s")
            self.move_vel = 0.20
        else:
            return
        
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
    
    def object_detected_cb(self, data):
        if not self.initialized:
            return
        self.detected_object = data.object_type
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
            
    def qr_detected_cb(self,data):
        if not self.initialized:
            return
        self.find_object = data.data
        if data.data == 1:
            rospy.loginfo("Finding blue object ")
        elif data.data == 2:
            rospy.loginfo("Finding orange object ")
        elif data.data == 3:
            rospy.loginfo("Finding green object")
        elif data.data == 4:
            rospy.loginfo("Finding red object")        

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
            y13 = int(height * 1 / 3)  # Only consider the bottom 1/3 of the image
            y12 = int(height * 1 / 2)  # Only consider the bottom 1/2 of the image
            mask[0:y12, :] = 0  # Set the top part of the mask to zero


            # #also set the bottom part of the mask to zero
            # mask[y13:height, :] = 0  # Set the bottom part of the mask to zero



            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            centroids = []
            
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
            cv2.imshow("Mask", mask)
            # cv2.imshow("Counters Image", contours_image)
            if self.record_video and self.video_writer is not None:
                frame_resized = cv2.resize(undis_image, self.video_size)  # Resize if needed
                self.video_writer.write(frame_resized)

            self.centroids = centroids
            cv2.waitKey(1)  # Non-blocking update

        except CvBridgeError as err:
            rospy.logerr("Error converting image: {}".format(err))
            return


# <=================== Motor control =========================>
    def timer_cb(self, event):
        if not self.initialized:
            return

        if self.robot_enabled: # Only execute if the robot is enabled
            if self.find_object == self.detected_object:
                rospy.loginfo_throttle(1.0, "Expected object has been detected. Stopping movement")
                self.call_stop()
                return
            if self.best_centroid is None:
                rospy.loginfo_throttle(2.0, "No line detected recently. No movement.")
                if self.centroids[0] == None:
                    self.best_centroid = self.centroids[0]
                else:
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

            # rospy.loginfo("Error: %.2f\tVLeft: %.2f\tVRight: %.2f", error, velocity_left, velocity_right)
            if velocity_left == self.max_speed:
                velocity_right = -velocity_right

            if velocity_right == self.max_speed:
                velocity_left = -velocity_left

            # Send motor commands
            self.call_left_wheel(dir_left, velocity_left)
            self.call_right_wheel(dir_right, velocity_right)
            time.sleep(0.01) 
            self.call_left_wheel(dir_left, 0)
            self.call_right_wheel(dir_right, 0)

            rospy.loginfo("Error: %.2f\tVLeft: %.2f\tVRight: %.2f", error, velocity_left, velocity_right)
        else:
            rospy.loginfo("Robot not initialized!")


    # ================ Motor command services ======================== #
    def call_move_straight(self, distance: float, speed: float) -> None:
        rospy.wait_for_service('move_straight')
        try:
            proxy = rospy.ServiceProxy('move_straight', MoveStraight)
            resp = proxy(distance, speed)
            print("MoveStraight success:", resp.success)
        except rospy.ServiceException as e:
            print("Service call failed:", e)

    def call_rotate(self, angle: float, angular_speed: float) -> None:
        rospy.wait_for_service('rotate')
        try:
            proxy = rospy.ServiceProxy('rotate', Rotate)
            resp = proxy(angle, angular_speed)
            print("Rotate success:", resp.success)
        except rospy.ServiceException as e:
            print("Service call failed:", e)

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
        if self.video_writer is not None:
            self.video_writer.release()
            rospy.loginfo(f"Video saved to {self.video_filename}")
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
    