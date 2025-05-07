#!/usr/bin/env python2

import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
from jetson_camera.msg import twovids, test


class CameraProcessorNode:
    def __init__(self):
        self.initialized = False
        rospy.loginfo("Initializing camera processor node...")
        self.bridge = CvBridge()

        # test variables
        #self.h = 0
        
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
            test,
            queue_size=1
        )

        self.first_image_received = False
        self.initialized = True

        self.camera_matrix = np.array([
            [438.7042285, 0.0, 328.47719321],
            [0.0, 582.21560648, 184.98541205],
            [0.0, 0.0, 1.0]
        ])
        self.distortion_coeff = np.array([
            -0.36759883, 0.15256109, 0.00324047, -0.00128645, -0.02882766
        ])

        self.first_image_received = False
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
            
            # ---------- DISPLAY SIDE BY SIDE --------------
            # Resize if needed to ensure same size (in case of different resolutions)
            #if cv_image.shape != cv_image_undistorted.shape:
            cv_image_undistorted = cv2.resize(cv_image_undistorted, (cv_image.shape[1], cv_image.shape[0]))

            self.publish_new_msg(cv_image, cv_image_undistorted)

            # Stack the images horizontally
            #side_by_side = np.hstack((cv_image, cv_image_undistorted))

            # Add a label to each half (optional)
            #cv2.putText(side_by_side, "Original", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
            #cv2.putText(side_by_side, "Undistorted", (cv_image.shape[1] + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

            # Display the result
            #cv2.imshow("Original vs. Undistorted", side_by_side)
            #cv2.waitKey(1)  # Non-blocking update
            
        except CvBridgeError as err:
            rospy.logerr("Error converting image: {}".format(err))

    def publish_new_msg(self, dis_img, undis_img):
        self.h = self.h + 1
        msg = twovids()
        #mesg = test()
        msg.raw_img.format = "jpeg"
        msg.undist_img.format = "jpeg"
        #mesg.num = self.h

        # Convert the OpenCV image to a ROS CompressedImage message
        success1, encoded_dis_image = cv2.imencode(".jpg", dis_img)
        success2, encoded_undis_image = cv2.imencode(".jpg", undis_img)
        if success1 and success2:
            #rospy.loginfo("Publish frame.")
            msg.raw_img.header.stamp = rospy.Time.now()
            msg.raw_img.data = encoded_dis_image.tobytes()
            
            msg.undist_img.header.stamp = rospy.Time.now()
            msg.undist_img.data = encoded_undis_image.tobytes()

            # Publish the image
            #rospy.loginfo("h: {num}".format(num=self.h))
            self.pub_image.publish(msg)

    def cleanup(self):
        cv2.destroyAllWindows()

# Main function run the node
if __name__ == "__main__":
    # Initialize the nodes
    rospy.init_node('camera_processor_node', anonymous=True, xmlrpc_port=45100, tcpros_port=45101)
    camera_node = CameraProcessorNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down image viewer node.")
    finally:
        camera_node.cleanup()
