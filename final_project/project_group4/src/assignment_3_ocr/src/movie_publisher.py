#!/usr/bin/env python2

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage


class CameraPublisherNode:
    def __init__(self, node_name):
        self.initialized = False
        rospy.loginfo("Initializing camera publisher node...")
        # Initialize the ROS node
        self.node_name = node_name
        rospy.init_node(self.node_name, anonymous=True)

        # Publisher for the image topic
        self.image_pub = rospy.Publisher(
            '/camera/image_raw', 
            CompressedImage,
            queue_size=1)

        # Create a CvBridge object for converting images
        self.bridge = CvBridge()

        # Load parameters
        self.video_path = rospy.get_param("~video_path", "/home/ubuntu/video.mp4")  # Default fallback
        self.fps = rospy.get_param("~fps", 30)
        self.loop = rospy.get_param("~loop", False)


        # Open the video file
        self.cap = cv2.VideoCapture(self.video_path)
        if not self.cap.isOpened():
            rospy.logerr("Unable to open video file: %s", self.video_path)
        else:
            rospy.loginfo("Video file loaded: %s", self.video_path)

        self.initialized = True


        self.first_image_received = False
        self.initialized = True
        rospy.loginfo("Camera publisher node initialized!")

    def start_publishing(self):
        rate = rospy.Rate(self.fps)
        msg = CompressedImage()
        msg.format = "jpeg"

        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret and not self.first_image_received :
                rospy.logerr("Failed to capture image")
                continue
            if self.first_image_received == False:
                self.first_image_received = True
                rospy.loginfo("Camera publisher captured first image.")
            if not ret and self.loop:
                rospy.loginfo("End of video reached. Restarting video...")
                self.cap.release()
                self.cap = cv2.VideoCapture(self.video_path)
                if not self.cap.isOpened():
                    rospy.logerr("Failed to reopen video file.")
                    break
                rospy.sleep(0.1)
                continue
            if not ret and not self.loop:
                rospy.loginfo("End of video reached.")
                break
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
        rospy.loginfo("Video file publisher node shut down.")


if __name__ == "__main__":
    node_name = "video_publisher_node"
    video_pub = None
    try:
        video_pub = CameraPublisherNode(node_name)
        video_pub.start_publishing()
    except rospy.ROSInterruptException:
        pass
    finally:
        if video_pub is not None:
            video_pub.cleanup()
