#!/usr/bin/env python2

import cv2
import rospy
import numpy as np
import pytesseract
import re
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
from assignment_3_ocr.msg import twovids
from motor_control.msg import motor_cmd
import easyocr 
import time
import torch
from motor_control.srv import MoveStraight, Rotate, Stop, ConstRotate, ConstStraight, DriveLeftwheel, DriveRightwheel, SetMaxSpeed

# from assignment_3_ocr.msg import motor_cmd
# from jetson_camera.msg import twovids, motor_cmd
class CameraSubscriberNode:
    def __init__(self):
        self.initialized = False
        rospy.loginfo("Initializing camera subscriber node...")
        self.bridge = CvBridge()

        self.writer = None
        self.first_image = True

        # Construct subscriber
        self.sub_image = rospy.Subscriber(
            "/camera/image_raw",
            CompressedImage,
            self.image_cb,
            buff_size=2**24,
            queue_size=1
        )
        # Publisher for sending movement commands
        self.pub_cmd = rospy.Publisher(
            "/motor_control",
            motor_cmd,
            queue_size = 10
        )
        self.prev_speed = None

        self.change_aspect = True

        self.first_image_received = False
        
        # self.video_save = True

        self.video_save = rospy.get_param("~video_save", False)
        self.confidence = rospy.get_param("~confidence", 0.4)
        self.commando_execution = rospy.get_param("~commando_execution", True)
        self.state = rospy.get_param("~state", 1)
        # self.commando_to_execute = 0
        self.prev_msg = motor_cmd()
        # zero messsage
        self.zero_msg = motor_cmd()
        self.zero_msg.velocity = 0
        self.zero_msg.angle = 0
        self.zero_msg.distance = 0
        # create message counter
        self.message_counter = 0
        self.reader = easyocr.Reader(['en'])
        # self.reader = easyocr.Reader(['en'], gpu=False)

        self.first_time = True       

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
            # raw_image = cv2.imdecode(np.frombuffer(data.data, np.uint8), cv2.IMREAD_COLOR)
            undis_image = cv2.imdecode(np.frombuffer(data.data, np.uint8), cv2.IMREAD_COLOR)
            # undis_image = cv2.imdecode(np.frombuffer(data.data, np.uint8), cv2.IMREAD_COLOR)

            # make copy 
            cv_image_2 = undis_image.copy()

            if self.change_aspect:
                undis_image = self.resize_with_aspect_ratio(undis_image, 1080)
            # execute OCR function
            # cv_image_processed = self.OCR_on_img(cv_image_2,self.confidence)
            
            cv_image_processed = self.EasyOCR_on_img(cv_image_2,self.confidence)
            if self.change_aspect:
                cv_image_processed = self.resize_with_aspect_ratio(cv_image_processed, 1080)

            # # Stack the images horizontally
            # side_by_side = np.hstack((undis_image, cv_image_processed))

            # # Add a label to each half (optional)
            # cv2.putText(side_by_side, "Original", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
            # cv2.putText(side_by_side, "OCR_processed", (cv_image_processed.shape[1] + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

            # Display the result
            cv2.imshow("Original vs. OCR", cv_image_processed)
            
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

        # OCR to commando function
        if self.commando_execution:
            cv_image_2 = self.OCR_to_command(data,self.state,cv_image_2)

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
                    if re.search(r'[A-Za-z0-9;:]', text):
                        # display the confidence and text to our terminal
                        rospy.loginfo("Confidence: {}".format(conf))
                        rospy.loginfo("Text: {}".format(text))
                        rospy.loginfo("")
                        # draw bounding box with text on screen
                        cv2.rectangle(cv_image_2, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        cv2.putText(cv_image_2, text, (x, y - 20), cv2.FONT_HERSHEY_SIMPLEX,
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

    def EasyOCR_on_img(self, cv_image_2,minimal_conf):
        # if self.first_time:
        #     print("CUDA available:", torch.cuda.is_available())
        #     print("Device count:", torch.cuda.device_count())
        #     print("Device name:", torch.cuda.get_device_name(0) if torch.cuda.is_available() else "No GPU")
        #     self.first_time = False
        start = time.time()
        results = self.reader.readtext(cv_image_2)
        if self.commando_execution: 
            cv_image_2 = self.OCR_to_command_text(results,minimal_conf,cv_image_2)

        for (bbox, text, confidence) in results:
            if confidence >= minimal_conf:
                if re.search(r'[A-Za-z0-9;:]', text):
                    rospy.loginfo(f"Detected text: {text} (Confidence: {confidence:.2f})")
                    pts = np.array(bbox, dtype=np.int32)
                    cv2.polylines(cv_image_2, [pts], isClosed=True, color=(0, 255, 0), thickness=2)
                    # cv2.rectangle(cv_image_2, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(cv_image_2, text, (pts[0]- 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 0, 255), 2)


        end = time.time()
        print(f"Elapsed time: {end - start:.3f} seconds")
        return cv_image_2

    def OCR_to_command(self,data,state,cv_image_2):
        # set msg
        msg = motor_cmd()
        # set variables
        keyword_no_colon = None
        # all accepted commando's 
        commando_executions = ["speed:", "move:", "angle:","speed", "move", "angle"]
        commando_to_execute = None
        # main loop
        for i in range(0, len(data["text"])):

            x = data["left"][i]
            y = data["top"][i]
            w = data["width"][i]
            h = data["height"][i]

            text = data["text"][i]
            conf = int(data["conf"][i])

            # filter out weak confidence text
            if conf > self.confidence:
                    # delete all non ascii characters
                    text = "".join([c if ord(c) < 128 else "" for c in text]).strip().lower()
                    # check if a minimal of 1 charcter from the alpahbet or nummers 1-9 is detected
                    if re.search(r'[A-Za-z0-9;:]', text):
                        #commando's are only executed if a colon is deteced after a key word 
                        if keyword_no_colon:
                            if re.search(r':', text):
                                commando_to_execute = keyword_no_colon + ':'
                                keyword_no_colon = None
                            else: 
                                rospy.loginfo("No colon found: {}".format(text))

                        if commando_to_execute:
                            match = re.search(r'[-+]?\d*\.\d+|[-+]?\d+', text)
                            if match:
                                val = float(match.group())
                                if commando_to_execute == "speed:":
                                   # speed case
                                    if val <0.1: 
                                        val = 0.5
                                    msg.velocity = val
                                    # self.pub_cmd.publish(msg)
                                    rospy.loginfo("set speed to: {}".format(val))

                                if commando_to_execute == "angle:":
                                   # angle case
                                    msg.angle = float(np.deg2rad(val))
                                    # self.pub_cmd.publish(msg)
                                    rospy.loginfo("set angle to: {}".format(val))                                   

                                if commando_to_execute == "move:":
                                   # move case
                                    msg.distance = val
                                    # self.pub_cmd.publish(msg)
                                    rospy.loginfo("set move to: {}".format(val))

                                commando_to_execute = None 

                            else:
                                rospy.loginfo("No valid number found: {}".format(text))
                        if text in commando_executions: 
                            # with colon
                            if re.search(r':', text):
                                commando_to_execute = text
                            else:
                                keyword_no_colon = text
                            cv2.rectangle(cv_image_2, (x-4, y-4), (x + w + 4, y + h + 4 ), (255,0 , 0), 2)
        # execute publish message here so all commandos get stacked and only once per image will commandos be executed
        # rospy.loginfo("zero message: {}".format(self.zero_msg ))
        # rospy.loginfo(" -- message: {}".format(msg))
        msg.new_mesg = self.message_counter
        self.zero_msg.new_mesg = self.message_counter
        if msg != self.zero_msg:   
            if msg != self.prev_msg:
                if msg.velocity<0.1:
                    msg.velocity = 0.5
                rospy.loginfo("Message Sent: {}".format(msg))
                self.pub_cmd.publish(msg)
                self.message_counter +=1
                msg.new_mesg = self.message_counter
                self.prev_msg = msg

                # rospy.loginfo("prev message: {}".format(self.prev_msg ))

        return cv_image_2
    
    def OCR_to_command_text(self,results,minimal_conf,cv_image_2):
        # set variables
        keyword_no_colon = None
        # all accepted commando's 
        commando_executions = ["speed:", "speed;","speed", "move", "angle"]
        commando_to_execute = None
        # main loop
        for (bbox, raw_text, confidence) in results: 

            # Match words, numbers, and single special characters
            
            if confidence >= minimal_conf:

                if re.search(r'[A-Za-z0-9;:]', raw_text):

                    # # delete all non ascii characters and make sure every lettel is lowercase
                    raw_text = "".join([c if ord(c) < 128 else "" for c in raw_text]).strip().lower()

                    raw_text = raw_text.replace(';', ':')
                    list_text = re.findall(r'\d+\.\d+|\d+|\w+|[^\s\w]', raw_text)
                    print(list_text)
                    print(type(list_text))

                    for i in range(0, len(list_text)):
                        
                        text = list_text[i]
                        #commando's are only executed if a colon is deteced after a key word 
                        if keyword_no_colon:
                            if re.search(r':', text):
                                commando_to_execute = keyword_no_colon + ':'
                                keyword_no_colon = None
                            else: 
                                rospy.loginfo("No colon found: {}".format(text))

                        if commando_to_execute:
                            match = re.search(r'[-+]?\d*\.\d+|[-+]?\d+', text)
                            if match:
                                # val = float(match.group())
                                num_str = match.group()
                                if re.fullmatch(r'0[1-9]', num_str):  # '01', '02', ..., '09'
                                    val = float('0.' + num_str[1])
                                else:
                                    val = float(num_str)
                                if commando_to_execute == "speed:":
                                    # speed case
                                    if val <0.1: 
                                        val = 0.5

                                    if val > 1.0: 
                                        val = 1.0
                                    
                                    if self.prev_speed != val:
                                        self.prev_speed = val
                                        # self.change_speed(val)
                                        # self.pub_cmd.publish(msg)
                                        rospy.loginfo("change speed to: {}".format(val))
                                    else: 
                                        rospy.loginfo("speed stays:{}".format(val))


                                if commando_to_execute == "angle:":
                                    # angle case
                                    msg.angle = float(np.deg2rad(val))
                                    # self.pub_cmd.publish(msg)
                                    rospy.loginfo("set angle to: {}".format(val))                                   

                                if commando_to_execute == "move:":
                                    # move case
                                    msg.distance = val
                                    # self.pub_cmd.publish(msg)
                                    rospy.loginfo("set move to: {}".format(val))

                                commando_to_execute = None 

                            else:
                                rospy.loginfo("No valid number found: {}".format(text))

                        if text in commando_executions: 
                            # with colon
                            print("also 3")
                            if re.search(r':', text):
                                commando_to_execute = text
                            else:
                                keyword_no_colon = text
                            pts = np.array(bbox, dtype=np.int32)
                            cv2.polylines(cv_image_2, [pts], isClosed=True, color=(255, 0, 0), thickness=3)

        return cv_image_2
    


    def resize_with_aspect_ratio(self,image, target_width):
        # Get the original dimensions
        original_height, original_width = image.shape[:2]
        
        # Define new width while maintaining the aspect ratio
        aspect_ratio = float(target_width) / original_width
        new_height = int(original_height * aspect_ratio)  # Compute height based on aspect ratio
        
        # Resize the image
        resized_image = cv2.resize(image, (target_width, new_height))

        return resized_image

    def change_speed(self, speed: float) -> None:
        rospy.wait_for_service('set_max_speed')
        try:
            proxy = rospy.ServiceProxy('set_max_speed', SetMaxSpeed)
            resp = proxy(speed)
            print("Set max speed command success:", resp.success)
        except rospy.ServiceException as e:
            print("Service call failed:", e)

    def cleanup(self):
        cv2.destroyAllWindows()

if __name__ == "__main__":
    # Initialize the node
    rospy.init_node('Motor_OCR_node', anonymous=False, xmlrpc_port=45102, tcpros_port=45103)
    camera_node = CameraSubscriberNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down Motor_OCR_node.")
    finally:
        camera_node.cleanup()
