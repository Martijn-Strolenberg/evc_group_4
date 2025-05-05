#!/usr/bin/env python2

import cv2
import rospy
#import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from rospy import TransportHints


class CameraSubscriberNode:
    def __init__(self):
        self.initialized = False
        rospy.loginfo("Initializing camera subscriber node...")
        
        self.topic_name = '/camera/image_raw'
        self.window_name = "Camera View"

        self.bridge = CvBridge()
        
        # Construct subscriber
        self.sub_image = rospy.Subscriber(
            self.topic_name,
            Image,
            self.image_cb,
            queue_size=1,
            transport_hints=TransportHints().tcp().tcp_no_delay(True)
        )

        self.first_image_received = False
        self.initialized = True
        rospy.loginfo("Camera subscriber node initialized!")

    def image_cb(self, data):
        if not self.initialized:
            return
        
        if not self.first_image_received:
            self.first_image_received = True
            rospy.loginfo("Camera subscriber captured first image from publisher.")
        try:
            cv_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: %s", e)
            return

        cv2.imshow(self.window_name, cv_img)
        cv2.waitKey(1)

    def cleanup(self):
        rospy.loginfo("Shutting down camera subscriber, closing window.")
        cv2.destroyAllWindows()

if __name__ == "__main__":
    # Initialize the node
    rospy.init_node('camera_subscriber_node', anonymous=True)
    camera_node = CameraSubscriberNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down image viewer node.")
    finally:
        camera_node.cleanup()
