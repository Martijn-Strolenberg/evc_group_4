#!/usr/bin/env python3

import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
from motor_control.srv import MoveStraight, Rotate, Stop, ConstRotate, ConstStraight, DriveLeftwheel, DriveRightwheel


class CameraSubscriberNode:
    def __init__(self):
        self.initialized = False
        rospy.loginfo("Initializing camera line follower node...")
        self.bridge = CvBridge()
        
        # Construct subscriber
        self.sub_image = rospy.Subscriber(
            "/camera/image_proc",
            CompressedImage,
            self.image_cb,
            buff_size=2**24,
            queue_size=1
        )

        self.cmd_rate = 10.0 # generate move commands at 10 Hz
        self.cmd_dt = 1.0 / self.cmd_rate
        self.last_cmd_ts = rospy.Time.now()
        self.timer = rospy.Timer(rospy.Duration(self.cmd_dt), self.timer_cb)

        # Line detection parameters    
        self.roi_height = 300                  # how many rows from the bottom to look at
        self.min_contour_area = 80            # discard tiny blobs
        self.lower_white = np.array([0, 0, 180], dtype=np.uint8)
        self.upper_white = np.array([150, 30, 255], dtype=np.uint8)
        self.latest_center = None               # (x, y) of the line centroid in full image coords
        self.latest_angle = None                # This will hold the last detected line angle (in radians), or None if no line was found
        self.zone_gains = [8.0, 4.0, 1.0, 4.0, 8.0]
        self.img_width = 640                   # we assume a 640×480 camera
        self.middle_x = self.img_width / 2.0   # center column = 320 px
        
        # Motor control parameters
        self.middle = 640 / 2                  # image center in x
        self.base_Kp = 0.005                          # steering gain (rad/s per pixel)
        self.v_base = 0.15                     # forward speed (m/s)
        self.B = 0.163                         # wheelbase (m)
        self.max_omega = 4.0                   # rad/s clamp
            
        rospy.on_shutdown(self.shutdown) # Shutdown hook to clean up resources

        self.initialized = True
        rospy.loginfo("Camera line follower node initialized!")

# <============== Image Processing =================>
    def image_cb(self, data):
        if not self.initialized:
            return

        try:
            # 1) Decode the compressed ROS image → BGR array
            full_bgr = cv2.imdecode(
                np.frombuffer(data.data, np.uint8),
                cv2.IMREAD_COLOR
            )
            height, width = full_bgr.shape[:2]

            # 2) Crop the bottom ROI of height self.roi_height
            y0 = max(0, height - self.roi_height)
            roi = full_bgr[y0:height, 0:width]

            # 3) Convert ROI → HSV and threshold for white
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, self.lower_white, self.upper_white)

            # 4) Remove small speckles via open/close
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            # 5) Find contours in that mask
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            best_centroid = None
            best_area = 0

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < self.min_contour_area:
                    continue

                M = cv2.moments(cnt)
                if M["m00"] == 0:
                    continue

                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                # Convert from ROI coords → full‐image coords:
                full_cx = cx
                full_cy = cy + y0

                if area > best_area:
                    best_area = area
                    best_centroid = (full_cx, full_cy)

            # Save the best centroid (or None)
            self.latest_center = best_centroid

            # 6) Debug: draw ROI box + centroid dot
            debug = full_bgr.copy()
            cv2.rectangle(debug, (0, y0), (width, height), (0, 0, 255), 2)

            if best_centroid is not None:
                cv2.circle(debug, best_centroid, 5, (0, 255, 0), -1)

                # Also draw the five vertical zones on the debug view:
                zone_w = width // 5
                for z in range(1, 5):
                    cv2.line(debug, (z * zone_w, 0), (z * zone_w, height), (200, 200, 0), 1)
                    cv2.putText(debug, f"Z{z}", (z * zone_w + 5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 0), 2)

                # Label which zone the centroid is in
                zone_idx = int(best_centroid[0] // zone_w)
                zone_idx = max(0, min(zone_idx, 4))
                cv2.putText(debug, f"Zone={zone_idx}", (best_centroid[0] + 10, best_centroid[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            else:
                cv2.putText(debug, "No line detected", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

            cv2.imshow("Line‐Follower Debug View (5-Zone)", debug)
            cv2.waitKey(1)

        except CvBridgeError as err:
            rospy.logerr("Error converting image: {}".format(err))
            return


# <=================== Motor control =========================>
    def timer_cb(self, event):
        if not self.initialized:
            return

        # If no centroid found, stop the robot
        if self.latest_center is None:
            rospy.loginfo_throttle(2.0, "No line → stopping.")
            self.call_stop()
            return

        # 1) Pixel‐error from center
        cx, _ = self.latest_center
        px_error = float(cx - self.middle_x)

        # 2) Find which of the 5 zones this column falls into
        zone_width = self.img_width / 5.0      # e.g. 640/5 = 128 px
        zone_idx = int(cx // zone_width)
        zone_idx = max(0, min(zone_idx, 4))    # clamp into [0..4]

        # 3) Pick the zone‐dependent gain
        zone_gain = self.zone_gains[zone_idx]
        gain = self.base_Kp * zone_gain

        # 4) Compute angular velocity (rad/s)
        omega = gain * px_error
        omega = float(np.clip(omega, -self.max_omega, self.max_omega))

        # 5) Forward velocity (constant)
        v = self.v_base

        # 6) Convert left/right wheel speeds
        v_r = v - (self.B / 2.0) * omega
        v_l = v + (self.B / 2.0) * omega

        # 7) Convert into (direction, speed) pairs for your services
        dir_r = 1 if v_r >= 0 else -1
        dir_l = 1 if v_l >= 0 else -1
        speed_r = abs(v_r)
        speed_l = abs(v_l)

        # 8) Send the service calls
        self.call_left_wheel(dir_l, speed_l)
        self.call_right_wheel(dir_r, speed_r)


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
    