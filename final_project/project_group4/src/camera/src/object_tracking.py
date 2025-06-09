#!/usr/bin/env python2

import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
from camera.srv import ObjectDetected
from motor_control.srv import Stop, ConstRotate, ConstStraight
from camera.msg import ObjectDetection


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

        self.timer = rospy.Timer(rospy.Duration(self.cmd_dt), self.timer_cb)

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

            # Define HSV ranges for orange, blue, red
            color_ranges = {
                #"orange": (np.array([5, 150, 150]), np.array([20, 255, 255])),
                "blue": (np.array([100, 100, 40]), np.array([130, 255, 255])),
                "green": (np.array([35, 40, 25]), np.array([85, 255, 255])),
                # Red has two ranges due to HSV wrapping
                "red_lower": (np.array([0, 120, 70]), np.array([10, 255, 255])),
                "red_upper": (np.array([170, 120, 70]), np.array([180, 255, 255])),
            }

            # Create masks for each color
            #mask_orange = cv2.inRange(hsv, *color_ranges["orange"])
            mask_blue = cv2.inRange(hsv, *color_ranges["blue"])
            mask_green = cv2.inRange(hsv, *color_ranges["green"])

            # For red, combine two masks
            mask_red_lower = cv2.inRange(hsv, *color_ranges["red_lower"])
            mask_red_upper = cv2.inRange(hsv, *color_ranges["red_upper"])
            mask_red = cv2.bitwise_or(mask_red_lower, mask_red_upper)

            # Morphological operations to clean noise for all masks
            kernel = np.ones((5, 5), np.uint8)
            # mask_orange = cv2.morphologyEx(mask_orange, cv2.MORPH_OPEN, kernel)
            # mask_orange = cv2.morphologyEx(mask_orange, cv2.MORPH_CLOSE, kernel)

            mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel)
            mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, kernel)

            mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)
            mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel)

            mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
            mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)

            # Helper function to detect largest contour center for a mask
            def detect_color_center(mask):
                contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
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


            # Update latest centers
            # if orange_res:
            #     self.latest_centers["orange"] = orange_res[0]
            #     x, y, w, h = orange_res[1]
            #     cv2.rectangle(undis_image, (x, y), (x + w, y + h), (0, 140, 255), 2)  # Orange box
            #     cx, cy = orange_res[0]
            #     cv2.line(undis_image, (cx - 5, cy), (cx + 5, cy), (0, 140, 255), 2)
            #     cv2.line(undis_image, (cx, cy - 5), (cx, cy + 5), (0, 140, 255), 2)
            # else:
            #     self.latest_centers["orange"] = None

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

            # Add a label to each half (optional)
            cv2.putText(undis_image, "Object Tracking", (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
            # cv2.putText(side_by_side, "Undistorted", (undis_image.shape[1] + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

            # Display the results
            cv2.putText(undis_image, "Object Tracking: Blue, Green, Red", (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.imshow("Object tracking", undis_image)
            # cv2.imshow("Orange Mask", mask_orange)
            cv2.imshow("Blue Mask", mask_blue)
            cv2.imshow("Red Mask", mask_red)
            cv2.imshow("Green Mask", mask_green)

            
            cv2.waitKey(1)  # Non-blocking update

        except CvBridgeError as err:
            rospy.logerr("Error converting image: {}".format(err))
            return


# <=================== Motor control =========================>
    def timer_cb(self, event):
        if not self.initialized:
            return
        
        # if self.latest_center is None:
        #     rospy.loginfo_throttle(2.0, "No object detected")
        #     #self.call_objdet(0)
        #     return
        # else: 
        #     rospy.loginfo_throttle(2.0, "Objected Detected")  
        #     #self.call_objdet(1)

        # Example logic: print detected centers for each color
        for color, center in self.latest_centers.items():
            if center is None:
                rospy.loginfo_throttle(2.0, "No {} object detected".format(color))
            else:
                #rospy.loginfo_throttle(1.0, "{} object detected at {}".format(color, center))
                if color == "blue":
                    self.publish_object_detected(1, center[0], center[1])
                # elif color == "orange":
                #     self.publish_object_detected(2, center[0], center[1])
                elif color == "green":
                    self.publish_object_detected(3, center[0], center[1])
                elif color == "red":
                    self.publish_object_detected(4, center[0], center[1])
                else:
                    self.publish_object_detected(0, center[0], center[1])
                


        # Define thresholds for 3x3 grid regions in the image
        left_thresh = 214
        right_thresh = 426
        top_thresh = 160
        bottom_thresh = 320

        # <================= object is in the middle ==================>
        #if ((left_thresh <=  center_x <= right_thresh) and (bottom_thresh <= center_y <= top_thresh)):
        #    rospy.loginfo("Object is in the middle")
        #    self.call_stop()

        # # <================= Movement logic upper row =================>
        # if ((center_x < left_thresh) and (center_y < top_thresh)):
        #     rospy.loginfo("Object is in the left upper corner.")
        #     self.call_const_rotate(-1, 0.5)

        # elif ((left_thresh <= center_x < right_thresh) and (center_y < top_thresh)):
        #     rospy.loginfo("Object is in the middle up.")
        #     self.call_const_straight(1, 0.5)

        # elif ((center_x >= right_thresh) and (center_y < top_thresh)):
        #     rospy.loginfo("Object is in the right upper corner.")
        #     self.call_const_rotate(1, 0.5)

        # # <================= Movement logic middle row =================>
        # elif ((center_x < left_thresh) and (top_thresh <= center_y < bottom_thresh)):
        #     rospy.loginfo("Object is in the middle left.")
        #     self.call_const_rotate(-1, 0.5)

        # elif ((left_thresh <= center_x < right_thresh) and (top_thresh <= center_y < bottom_thresh)):
        #     rospy.loginfo("Object is in the middle.")
        #     self.call_stop()

        # elif ((center_x >= right_thresh) and (top_thresh <= center_y < bottom_thresh)):
        #     rospy.loginfo("Object is in the middle right.")
        #     self.call_const_rotate(1, 0.5)

        # # <================= Movement logic lower row =================>
        # elif ((center_x < left_thresh) and (center_y >= bottom_thresh)):
        #     rospy.loginfo("Object is in the left lower corner.")
        #     self.call_const_rotate(-1, 0.5)

        # elif ((left_thresh <= center_x < right_thresh) and (center_y >= bottom_thresh)):
        #     rospy.loginfo("Object is in the middle down.")
        #     self.call_const_straight(-1, 0.5)

        # elif ((center_x >= right_thresh) and (center_y >= bottom_thresh)):
        #     rospy.loginfo("Object is in right lower corner.")
        #     self.call_const_rotate(1, 0.5)

        # # this should never be possible
        # else:
        #     rospy.loginfo_throttle(2.0, "Object not found")
        #     return

    def publish_object_detected(self, object_type, x, y):
        msg = ObjectDetection()
        msg.object_type = object_type 
        msg.x_coordinate = x
        msg.y_coordinate = y
        self.pub_object_detected.publish(msg)
        rospy.loginfo("Published object detected of type: {} at location ({}, {})".format(object_type, x, y))


    # <================= Services =================>
    def call_objdet(self, obj: int) -> None:
        rospy.wait_for_service('object_detected')
        try:
            proxy = rospy.ServiceProxy('object_detected', ObjectDetected)
            resp = proxy(obj)
            print("Object command successfully detected:", resp.success)
        except rospy.ServiceException as e:
            print("Service call failed:", e)

    def call_const_rotate(direction, angular_speed):
        rospy.wait_for_service('const_rotate')
        try:
            proxy = rospy.ServiceProxy('const_rotate', ConstRotate)
            resp = proxy(direction, angular_speed)
            print("Constant Rotate success:", resp.success)
        except rospy.ServiceException as e:
            print("Service call failed:", e)

    def call_const_straight(direction, speed):
        rospy.wait_for_service('const_straight')
        try:
            proxy = rospy.ServiceProxy('const_straight', ConstStraight)
            resp = proxy(direction, speed)
            print("Moving Constant Straight success:", resp.success)
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

    def cleanup(self):
        cv2.destroyAllWindows()

if __name__ == "__main__":
    # Initialize the node
    rospy.init_node('object_detection_node', anonymous=False, xmlrpc_port=45104, tcpros_port=45105)
    camera_node = CameraSubscriberNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down image viewer node.")
    finally:
        camera_node.cleanup()
