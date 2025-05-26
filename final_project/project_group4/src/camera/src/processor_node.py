#!/usr/bin/env python2

import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage


class CameraProcessorNode:
    def __init__(self):
        self.initialized = False
        self.first_image_received = False
        rospy.loginfo("Initializing camera processor node...")
        self.bridge = CvBridge()
        
        # Construct subscriber
        self.sub_image = rospy.Subscriber(
            "/camera/image_raw",
            CompressedImage,
            self.image_cb,
            buff_size=2**24,
            queue_size=1
        )

        self.pub_image = rospy.Publisher(
            "/camera/image_proc",
            CompressedImage,
            queue_size=1
        )

        # Undistortion parameters
        self.camera_matrix = np.array([
            [438.7042285, 0.0, 328.47719321],
            [0.0, 582.21560648, 184.98541205],
            [0.0, 0.0, 1.0]
        ])
        self.distortion_coeff = np.array([
            -0.36759883, 0.15256109, 0.00324047, -0.00128645, -0.02882766
        ])

        self.initialized = True

    # Function that is executed every time it receives a new message from the publisher
    def image_cb(self, data):
        if not self.initialized:
            return
        
        if not self.first_image_received:
            self.first_image_received = True
            rospy.loginfo("Processor node initialized.")
        try:
            # Decode image without CvBridge
            cv_image = cv2.imdecode(np.frombuffer(data.data, np.uint8), cv2.IMREAD_COLOR)
            
            # ---------- UNDISTORTION --------------
            cv_image_undistorted = cv2.undistort(cv_image, self.camera_matrix, self.distortion_coeff)
            cv_image_undistorted = cv2.resize(cv_image_undistorted, (cv_image.shape[1], cv_image.shape[0]))

            self.publish_new_msg(cv_image_undistorted)
            
        except CvBridgeError as err:
            rospy.logerr("Error converting image: {}".format(err))

    def publish_new_msg(self, undis_img):
        msg = CompressedImage()
        msg.format = "jpeg"

        # Convert the OpenCV image to a ROS CompressedImage message
        success, encoded_undis_image = cv2.imencode(".jpg", undis_img)
        if success:
            msg.header.stamp = rospy.Time.now()
            msg.data = encoded_undis_image.tobytes()

            # Publish the image
            self.pub_image.publish(msg)

    def cleanup(self):
        cv2.destroyAllWindows()

# Main function run the node
if __name__ == "__main__":
    # Initialize the nodes
    rospy.init_node('camera_processor_node', anonymous=False, xmlrpc_port=45100, tcpros_port=45101)
    camera_node = CameraProcessorNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down camera processor node.")
    finally:
        camera_node.cleanup()
