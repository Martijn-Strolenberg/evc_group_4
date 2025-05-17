#!/usr/bin/env python2

import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
from jetson_camera.msg import twovids, motor_cmd, encoder


class personcounter:
    def __init__(self):
        self.initialized = False
        rospy.loginfo("initialising")
        self.bridge = CvBridge()

        self.writer = None
        self.first_image = True
        
        # Subscriber for recieving live images from camera
        self.sub_image = rospy.Subscriber( 
            "/camera/image_proc",
            twovids,
            self.image_cb,
            buff_size=2**24,
            queue_size=1
        )
        # Subscriber to check if robot is moving
        self.sub_move_status = rospy.Subscriber( 
            "/encoder",
            encoder,
            self.movement_detector,
            buff_size=2**24,
            queue_size=1
        )
        # Publisher for sending movement commands
        self.pub_cmd = rospy.Publisher(
            "/motor_control",
            motor_cmd,
            queue_size = 1
        )

        self.first_image_received = False
        self.initialized = True
        rospy.loginfo("initialised")

        # Movement init
        self.min_people = 1000000000 # large value to make sure
        self.min_position = -1 # Save position
        self.position_track = 0 # Track how many times 
        self.in_pos = 0.0 # If the robot has rotated to correct position
        self.action = 0 # What type of movement it is doing 


    def image_cb(self, data):
        if not self.initialized:
            return
        
        if not self.first_image_received:
            self.first_image_received = True
            rospy.loginfo("Camera subscriber captured first image from publisher.")
        try:
            #create haar feature map
            self.haar_objct = cv2.CascadeClassifier('src/jetson_camera/src/haarcascade_frontalface_alt2.xml')

            # decode the incoming images from custom messages
            raw_image = cv2.imdecode(np.frombuffer(data.raw_img.data, np.uint8), cv2.IMREAD_COLOR)
            undis_image = cv2.imdecode(np.frombuffer(data.undist_img.data, np.uint8), cv2.IMREAD_COLOR)

            # apply haar features
            gray_undis = cv2.cvtColor(undis_image,cv2.COLOR_BGR2GRAY)
            person = self.haar_objct.detectMultiScale(gray_undis,1.1,4)

            side_by_side = np.hstack((raw_image, undis_image))
            # Identification for which video is which along with live people counter
            cv2.putText(side_by_side, "Original", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.putText(side_by_side, "Number of people: " + str(len(person)), (undis_image.shape[1] + 10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

            cv2.imshow("Original vs undistored people counter", side_by_side)

            ## Motor control and Publisher
            self.send_mtrcmd(len(person))

            cv2.waitKey(1)

        except CvBridgeError as err:
            rospy.logerr("Error converting image: {}".format(err))
            return
        
    def movement_detector(self,data):
        self.action = data.move_cmd # Recieve action
    def send_mtrcmd(self,num_people):
        msg = motor_cmd()
        ## Make a full circle through 8*45 degrees
        ## Store the location with the lowest number of people
        ## After full circle. Turn to position with least number of peopl
        ## Move forward 1 meter. 
        ## Repeat
        ## If there is more than one camera view with the lowest number of people,
        #  go with the first one

        ## This way theoretically works/
        if self.action == 0: # If motors are not moving or have reached their

        # Check if new position has less people.
            if num_people < self.min_people and not self.in_pos:
                self.min_people = num_people
                self.min_position = self.position_track

            # iterate through each predefined position
            if self.position_track < 8 and not self.in_pos:
                self.position_track += 1
                msg.velocity = 0.3
                msg.angle = 45
                self.pub_cmd.publish(msg)

            # Move to the position which it detected least number of people
            if self.position_track == 8 and not self.in_pos:
                msg.angle = self.min_position*45
                msg.velocity = 0.3
                self.pub_cmd.publish(msg)
                self.in_pos = 1

            # If it has moved to the correct position move forward
            if self.in_pos:
                msg.distance = 1
                msg.velocity = 0.6
                self.pub_cmd.publish(msg)
                self.in_pos = 0

    def cleanup(self):
        cv2.destroyAllWindows()

if __name__ == "__main__":
    # Initialise node
    rospy.init_node('camera_viewer_people_counter', anonymous=True)
    camera_node = personcounter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        camera_node.cleanup()
