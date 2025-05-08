#!/usr/bin/env python2

import cv2
import rospy
from sensor_msgs.msg import CompressedImage

class MoviePublisherNode:
    def __init__(self, node_name):
        rospy.init_node(node_name, anonymous=True, xmlrpc_port=45100,tcpros_port=45101)
        rospy.loginfo("Initializing movie publisher node...")

        # Retrieve parameters
        self.movie_path = rospy.get_param("~movie_path", "/path/to/movie.mp4")
        self.fps = rospy.get_param("~fps", 25)

        # Initialize video capture
        self.cap = cv2.VideoCapture(self.movie_path)
        if not self.cap.isOpened():
            rospy.logerr("Failed to open video file: {}".format(self.movie_path))
            raise IOError("Cannot open video file")
        
        self.first_image_received = False


        # Publisher for the image topic
        self.image_pub = rospy.Publisher(
            '/camera/image_raw',
            CompressedImage,
            queue_size=1
        )

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
    movie_pub = None
    try:
        movie_pub = MoviePublisherNode(node_name)
        movie_pub.start_publishing()
    except rospy.ROSInterruptException:
        pass
    finally:
        if movie_pub is not None:
            movie_pub.cleanup()
