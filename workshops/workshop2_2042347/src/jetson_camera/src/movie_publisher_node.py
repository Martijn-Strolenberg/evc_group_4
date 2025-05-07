#!/usr/bin/env python2

import cv2
import rospy
from sensor_msgs.msg import CompressedImage
import os


class CameraPublisherNode:
    def __init__(self, node_name):
        self.initialized = False
        rospy.loginfo("Initializing movie publisher node...")
        # Initialize the ROS node
        self.node_name = node_name
        rospy.init_node(self.node_name, anonymous=True)

        # Publisher for the image topic
        self.image_pub = rospy.Publisher(
            '/movie/image_raw', 
            CompressedImage,
            queue_size=1)
        
        rospy.loginfo("Current working directory: {}".format(os.getcwd()))
        self.movie_path = os.path.join(os.path.dirname(__file__), "output.avi")
        self.fps = 10

        self.cap = cv2.VideoCapture(self.movie_path)
        if not self.cap.isOpened():
            rospy.logerr("Failed to open video file: {}".format(self.movie_path))
            raise IOError("Cannot open video file")


        self.first_image_received = False
        self.initialized = True
        rospy.loginfo("Movie publisher node initialized!")


    def start_publishing(self):
        rate = rospy.Rate(self.fps)
        msg = CompressedImage()
        msg.format = "jpeg"

        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                rospy.logwarn("End of video reached. Rewinding to start.")
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                continue

            if not self.first_image_received:
                self.first_image_received = True
                rospy.loginfo("Camera publisher captured first image from video.")

            try:
                success, encoded_image = cv2.imencode(".jpg", frame)
                if success:
                    msg.header.stamp = rospy.Time.now()
                    msg.data = encoded_image.tobytes()
                    self.image_pub.publish(msg)
            except CvBridgeError as e:
                rospy.logerr("Error converting image: {}".format(e))

            rate.sleep()

    def cleanup(self):
        self.cap.release()


if __name__ == "__main__":
    node_name = "movie_publisher_node"
    camera_pub = None
    try:
        camera_pub = CameraPublisherNode(node_name)
        camera_pub.start_publishing()
    except rospy.ROSInterruptException:
        pass
    finally:
        if camera_pub is not None:
            camera_pub.cleanup()
