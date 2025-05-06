#!/usr/bin/env python2

import cv2
import rospy
import time
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage


class CameraPublisherNode:
    def __init__(self, node_name):
        self.initialized = False
        rospy.loginfo("Initializing camera publisher node...")
        # Initialize the ROS node
        self.node_name = node_name
        rospy.init_node(self.node_name, anonymous=True)

        # --- parameters ---
        self.topic_name = '/camera/image_raw'
        self.stream_url = 'rtsp://host.docker.internal:8554/mystream'
        self.drop_frames = 5

        # Publisher for the image topic
        self.image_pub = rospy.Publisher(
            self.topic_name, 
            Image,
            queue_size=1)

        # Create a CvBridge object for converting images
        self.bridge = CvBridge()

        # OpenCV video capture
        self.cap = cv2.VideoCapture(self.stream_url)

        if not self.cap.isOpened():
            rospy.logerr("Failed to open stream: %s", self.stream_url)
            raise RuntimeError("Cannot open stream")

        # --- compute rate from stream FPS (fallback to 15) ---
        fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.publish_rate = rospy.Rate(fps if fps > 0 else 15.0)
        rospy.on_shutdown(self.cleanup)
        self.first_image_received = False
        self.initialized = True
        rospy.loginfo("CameraPublisherNode initialized.\n"
                      "  stream: %s\n"
                      "  topic:  %s\n"
                      "  fps:    %.2f",
                      self.stream_url, self.topic_name,
                      fps if fps > 0 else 15.0)

    def start_publishing(self):
        """Main loop: grab frames, convert, and publish."""
        rospy.loginfo("Starting capture loop.")
        while not rospy.is_shutdown():
            # drop stale frames
            for _ in range(self.drop_frames):
                self.cap.grab()

            ret, frame = self.cap.read()
            if not ret or frame is None:
                rospy.logwarn("Empty frame received, skipping.")
                self.publish_rate.sleep()
                continue

            try:
                msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                msg.header.stamp = rospy.Time.now()
                self.image_pub.publish(msg)
                rospy.logdebug("Published frame at %.3f", time.time())
            except CvBridgeError as e:
                rospy.logerr("CvBridge error: %s", e)

            self.publish_rate.sleep()

    def cleanup(self):
        """Release camera and do any other teardown."""
        rospy.loginfo("Shutting down CameraPublisherNode, releasing capture.")
        if self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    node_name = "camera_publisher_node"
    camera_pub = None
    try:
        camera_pub = CameraPublisherNode(node_name)
        camera_pub.start_publishing()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logfatal("Unhandled exception in CameraPublisherNode: %s", e)
    finally:
        if camera_pub is not None:
            camera_pub.cleanup()
