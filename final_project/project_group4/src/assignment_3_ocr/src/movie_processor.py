#!/usr/bin/env python2

import cv2
import rospy
import numpy as np
import pytesseract
import re
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
from assignment_2_ocr.msg import vids


class CameraProcessorNode:
    def __init__(self):
        self.initialized = False
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
            vids,
            queue_size=1
        )

        self.confidence = rospy.get_param("~confidence", 50)
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
            cv_image_2 = cv_image.copy()

            cv_image_processed = self.OCR_on_img(cv_image_2,self.confidence)
            self.publish_new_msg(cv_image, cv_image_processed)
            
        except CvBridgeError as err:
            rospy.logerr("Error converting image: {}".format(err))

    def publish_new_msg(self, raw_img, proc_img):

        msg = vids()

        msg.raw_img.format = "jpeg"
        msg.proc_img.format = "jpeg"


        # Convert the OpenCV image to a ROS CompressedImage message
        success1, encoded_raw_img = cv2.imencode(".jpg", raw_img)
        success2, encoded_proc_img = cv2.imencode(".jpg", proc_img)
        if success1 and success2:
            #rospy.loginfo("Publish frame.")
            msg.raw_img.header.stamp = rospy.Time.now()
            msg.raw_img.data = encoded_raw_img.tobytes()
            
            msg.proc_img.header.stamp = rospy.Time.now()
            msg.proc_img.data = encoded_proc_img.tobytes()

            # Publish the image
            self.pub_image.publish(msg)

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
