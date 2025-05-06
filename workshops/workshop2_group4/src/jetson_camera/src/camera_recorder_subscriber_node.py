#!/usr/bin/env python2

import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
from jetson_camera.msg import twovids


class CameraSubscriberNode:
    def __init__(self):
        self.initialized = False
        rospy.loginfo("Initializing camera subscriber node...")
        self.bridge = CvBridge()

        self.writer = None
        self.first_image = True
        
        # Construct subscriber
        self.sub_image = rospy.Subscriber(
            "/camera/image_proc",
            twovids,
            self.image_cb,
            buff_size=2**24,
            queue_size=1
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
            # Decode image without CvBridge
            #raw_image = cv2.imdecode(np.frombuffer(data.raw_img.data, np.uint8), cv2.IMREAD_COLOR)
            undis_image = cv2.imdecode(np.frombuffer(data.undist_img.data, np.uint8), cv2.IMREAD_COLOR)

            rospy.loginfo("Trying to show camera")
            # Ensure the window updates instantly
            cv2.imshow("Camera View", undis_image)
            cv2.waitKey(1)  # Keep at 1 to prevent blocking
        except CvBridgeError as err:
            rospy.logerr("Error converting image: {}".format(err))
            return
        """
        # On first frame only: set up VideoWriter
        if self.first_image:
            h, w = frame.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*"XVID")
            fps    = 15.0  # or read msg.header.stamp deltas
            self.writer = cv2.VideoWriter("output.avi",
                                          fourcc,
                                          fps,
                                          (w, h))
            if not self.writer.isOpened():
                rospy.logfatal("Cannot open VideoWriter")
                rospy.signal_shutdown("VideoWriter failed")
                return
            rospy.loginfo("Recording %dx%d @ %.1f FPS to output.avi", w, h, fps)
            self.first_image = False

        # Write the frame
        self.writer.write(frame)

        # Optional: display it
        cv2.imshow("Camera View", frame)
        cv2.waitKey(1)
        """

    def cleanup(self):
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
