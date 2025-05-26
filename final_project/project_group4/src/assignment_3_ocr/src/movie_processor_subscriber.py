#!/usr/bin/env python2

import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
from assignment_2_ocr.msg import vids


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
            vids,
            self.image_cb,
            buff_size=2**24,
            queue_size=1
        )

        self.first_image_received = False
        self.initialized = True
        self.video_save = rospy.get_param("~video_save", False)
        rospy.loginfo("Camera subscriber node initialized!")


    def image_cb(self, data):
        if not self.initialized:
            return
        
        if not self.first_image_received:
            self.first_image_received = True
            rospy.loginfo("Camera subscriber captured first image from publisher.")
        try:
            # Decode image without CvBridge
            raw_image = cv2.imdecode(np.frombuffer(data.raw_img.data, np.uint8), cv2.IMREAD_COLOR)
            proc_img = cv2.imdecode(np.frombuffer(data.proc_img.data, np.uint8), cv2.IMREAD_COLOR)

            # raw_image_resized =self.resize_with_aspect_ratio(raw_image, 700)
            # proc_img_resized = self.resize_with_aspect_ratio(proc_img, 700)

            # Stack the images horizontally
            side_by_side = np.hstack((raw_image, proc_img))

            # Add a label to each half (optional)
            cv2.putText(side_by_side, "Original", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.putText(side_by_side, "OCR_processed", (proc_img.shape[1] + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

            # Display the result
            # cv2.namedWindow("Original vs. OCR", cv2.WINDOW_NORMAL)
            cv2.imshow("Original vs. OCR", side_by_side)
            
            if self.video_save:
                # On first frame only: set up VideoWriter
                if self.first_image:
                    h, w = proc_img.shape[:2]
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
                self.writer.write(proc_img)

            #cv2.waitKey(1)
            cv2.waitKey(1)  # Non-blocking update

        except CvBridgeError as err:
            rospy.logerr("Error converting image: {}".format(err))
            return

    def resize_with_aspect_ratio(self,image, target_width):
        # Get the original dimensions
        original_height, original_width = image.shape[:2]
        
        # Define new width while maintaining the aspect ratio
        aspect_ratio = float(target_width) / original_width
        new_height = int(original_height * aspect_ratio)  # Compute height based on aspect ratio
        
        # Resize the image
        resized_image = cv2.resize(image, (target_width, new_height))

        return resized_image


    def cleanup(self):
        cv2.destroyAllWindows()

if __name__ == "__main__":
    # Initialize the node
    rospy.init_node('camera_viewer_node', anonymous=True)
    camera_node = CameraSubscriberNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down image viewer node.")
    finally:
        camera_node.cleanup()
