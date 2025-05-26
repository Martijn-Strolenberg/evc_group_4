#!/usr/bin/env python2

import cv2
import rospy
import numpy as np
import pytesseract
import re
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
from assignment_2_ocr.msg import twovids


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
        self.video_save = True

        self.video_save = rospy.get_param("~video_save", False)
        self.confidence = rospy.get_param("~confidence", 50)

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
            undis_image = cv2.imdecode(np.frombuffer(data.undist_img.data, np.uint8), cv2.IMREAD_COLOR)

            # make copy 
            cv_image_2 = undis_image.copy()
            # execute OCR function
            cv_image_processed = self.OCR_on_img(cv_image_2,self.confidence)


            # Stack the images horizontally
            side_by_side = np.hstack((raw_image, cv_image_processed))

            # Add a label to each half (optional)
            cv2.putText(side_by_side, "Original", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.putText(side_by_side, "OCR_processed", (cv_image_processed.shape[1] + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

            # Display the result
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

    #OCR on image function
    def OCR_on_img(self, cv_image_2,minimal_conf):
        
        #Thresholding 
        image_rgb = cv2.cvtColor(cv_image_2, cv2.COLOR_BGR2RGB)
        gray = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2GRAY)
        thresh_gaussian = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
        cv2.THRESH_BINARY, 23, 8)

        #get data from image, text and size of image
        height, width = cv_image_2.shape[:2]
        data = pytesseract.image_to_data(thresh_gaussian, output_type=pytesseract.Output.DICT)

        # loop over each word that was found
        for i in range(0, len(data["text"])):

            # extract the size and location of a word
            x = data["left"][i]
            y = data["top"][i]
            w = data["width"][i]
            h = data["height"][i]

            # extract the detected text and confidence level
            text = data["text"][i]
            conf = int(data["conf"][i])

            # filter out weak confidence text
            if conf > minimal_conf:
                # filter out too large texts
                if h < (height/2):
                    # strip out everything but ASCII text
                    text = "".join([c if ord(c) < 128 else "" for c in text]).strip()
                    # check if a minimal of 1 charcter from the alpahbet or nummers 1-9 is detected
                    if re.search(r'[A-Za-z0-9]', text):
                        # display the confidence and text to our terminal
                        rospy.loginfo("Confidence: {}".format(conf))
                        rospy.loginfo("Text: {}".format(text))
                        rospy.loginfo("")
                        # draw bounding box with text on screen
                        cv2.rectangle(cv_image_2, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        cv2.putText(cv_image_2, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (0, 0, 255), 2)

        return cv_image_2

    # Could be used to get the orientation of text and correct for possible rotations 
    def correct_rotation(self,image):
        # Tesseract OSD will give us the angle of rotation
        osd = pytesseract.image_to_osd(image, output_type=pytesseract.Output.DICT)
        angle = osd['rotate']

        if angle != 0:
            (h, w) = image.shape[:2]
            center = (w // 2, h // 2)
            M = cv2.getRotationMatrix2D(center, -angle, 1.0)
            image = cv2.warpAffine(image, M, (w, h),
            flags=cv2.INTER_CUBIC, borderMode=cv2.BORDER_REPLICATE)
        return image , angle

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
