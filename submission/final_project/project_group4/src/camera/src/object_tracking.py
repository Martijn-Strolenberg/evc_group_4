#!/usr/bin/env python3

import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
from camera.srv import ObjectDetected
from motor_control.srv import Stop, ConstRotate, ConstStraight
from camera.msg import ObjectDetection
from std_msgs.msg import UInt8

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

        self.pub_object_detected = rospy.Publisher(
            "/object_detection",
            ObjectDetection,
            queue_size=10
        )

        # self.obj_find = "orange"

        self.cmd_rate = 2.0 # generate move commands at 5Hz
        self.cmd_dt = 1.0 / self.cmd_rate
        self.last_cmd_ts = rospy.Time.now()
        # self.latest_center = None  # updated every frame
        self.new_cmd = 0 
        self.distance_cmd = 0.08
        self.angle_cmd = 0.3
        self.turn_vel = 0.5
        self.move_vel = 0.6

        self.latest_centers = {
            #"orange": None,
            "blue": None,
            "green": None,
            "red": None
        }

        self.timer = rospy.Timer(rospy.Duration(self.cmd_dt), self.obj_detected)

        self.initialized = True
        rospy.loginfo("Camera object detection node initialized!")

# <============== Image Processing =================>
    def image_cb(self, data):
        if not self.initialized:
            return

        try:
            # decode the image
            undis_image = cv2.imdecode(np.frombuffer(data.data, np.uint8), cv2.IMREAD_COLOR)

            # <================= START: Image Processing ======================>
            # Convert to HSV color space
            hsv = cv2.cvtColor(undis_image, cv2.COLOR_BGR2HSV)

            # Define HSV ranges for green, blue, red
            color_ranges = {

                #"blue": (np.array([105, 130, 60]), np.array([130, 255, 255])), # Less sensitive
                "blue": (np.array([100, 100, 40]), np.array([130, 255, 255])), # More sensitive

                "green": (np.array([50, 80, 50]), np.array([85, 255, 255])), # Less sensitive
                #"green": (np.array([35, 40, 25]), np.array([85, 255, 255])),# More sensitive

                #"red": (np.array([0, 150, 85]), np.array([70, 255, 255])), # less sensitive
                "red": (np.array([0, 150, 72]), np.array([70, 255, 255])), # More sensitive
            }

            # Create masks for each color
            mask_blue = cv2.inRange(hsv, *color_ranges["blue"])
            mask_green = cv2.inRange(hsv, *color_ranges["green"])
            mask_red = cv2.inRange(hsv, *color_ranges["red"])

            # Morphological operations to clean noise for all masks
            kernel = np.ones((5, 5), np.uint8)

            mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel)
            mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, kernel)

            mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)
            mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel)

            mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
            mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)

            #detect largest contour center for a mask
            def detect_color_center(mask):
                _, contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if not contours:
                    return None
                # Find the largest contour by area
                largest_contour = max(contours, key=cv2.contourArea)
                if cv2.contourArea(largest_contour) < 200:
                    return None
                x, y, w, h = cv2.boundingRect(largest_contour)
                center_x = x + w // 2
                center_y = y + h // 2
                return (center_x, center_y), (x, y, w, h)

            # Detect centers and bounding boxes for each color
            # orange_res = detect_color_center(mask_orange)
            blue_res = detect_color_center(mask_blue)
            green_res = detect_color_center(mask_green)
            red_res = detect_color_center(mask_red)

            if blue_res:
                self.latest_centers["blue"] = blue_res[0]
                x, y, w, h = blue_res[1]
                cv2.rectangle(undis_image, (x, y), (x + w, y + h), (255, 0, 0), 2)  # Blue box
                cx, cy = blue_res[0]
                cv2.line(undis_image, (cx - 5, cy), (cx + 5, cy), (255, 0, 0), 2)
                cv2.line(undis_image, (cx, cy - 5), (cx, cy + 5), (255, 0, 0), 2)
            else:
                self.latest_centers["blue"] = None
            
            if green_res:
                self.latest_centers["green"] = green_res[0]
                x, y, w, h = green_res[1]
                cv2.rectangle(undis_image, (x, y), (x + w, y + h), (0, 255, 0), 2)  # Green box
                cx, cy = green_res[0]
                cv2.line(undis_image, (cx - 5, cy), (cx + 5, cy), (0, 255, 0), 2)
                cv2.line(undis_image, (cx, cy - 5), (cx, cy + 5), (0, 255, 0), 2)
            else:
                self.latest_centers["green"] = None

            if red_res:
                self.latest_centers["red"] = red_res[0]
                x, y, w, h = red_res[1]
                cv2.rectangle(undis_image, (x, y), (x + w, y + h), (0, 0, 255), 2)  # Red box
                cx, cy = red_res[0]
                cv2.line(undis_image, (cx - 5, cy), (cx + 5, cy), (0, 0, 255), 2)
                cv2.line(undis_image, (cx, cy - 5), (cx, cy + 5), (0, 0, 255), 2)
            else:
                self.latest_centers["red"] = None
            # <===================== END: Image Processing =====================>

            # label 
            cv2.putText(undis_image, "Object Tracking", (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
            

            # Display the results
            cv2.putText(undis_image, "Object Tracking: Blue, Green, Red", (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.imshow("Object tracking", undis_image)
            
            #Display Masks
            #cv2.imshow("Blue Mask", mask_blue)
            #cv2.imshow("Red Mask", mask_red)
            #cv2.imshow("Green Mask", mask_green)

            
            cv2.waitKey(1)  # Non-blocking update

        except CvBridgeError as err:
            rospy.logerr("Error converting image: {}".format(err))
            return

# <=================== Detected ==============================>
    def obj_detected(self,event):
        msg = ObjectDetection()
        for color, center in self.latest_centers.items():
            if center is None:
                #rospy.loginfo_throttle(2.0, "No {} object detected".format(color))
                continue
            else:
                rospy.loginfo_throttle(1.0, "{} object detected at {}".format(color, center))

                msg.x_coordinate = center[0]
                msg.y_coordinate = center[1]
                if color == "blue":
                    msg.object_type = 1
                    self.pub_object_detected.publish(msg)
                elif color == "green":
                    msg.object_type = 3
                    self.pub_object_detected.publish(msg)
                elif color == "red":
                    msg.object_type = 4
                    self.pub_object_detected.publish(msg)

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
