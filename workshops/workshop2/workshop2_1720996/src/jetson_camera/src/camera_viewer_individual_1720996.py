#!/usr/bin/env python2

import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
from jetson_camera.msg import twovids


class personcounter:
    def __init__(self):
        self.initialized = False
        rospy.loginfo("initialising")
        self.bridge = CvBridge()

        self.writer = None
        self.first_image = True
        
        # make subscriber
        self.sub_image = rospy.Subscriber(
            "/camera/image_proc",
            twovids,
            self.image_cb,
            buff_size=2**24,
            queue_size=1
        )

        self.first_image_received = False
        self.initialized = True
        rospy.loginfo("initialised")


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
            
            cv2.waitKey(1)  

        except CvBridgeError as err:
            rospy.logerr("Error converting image: {}".format(err))
            return


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
