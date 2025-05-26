
import time
import hat
from hat import *

import cv2
import rospy
import numpy as np

from sensor_msgs.msg import CompressedImage
from jetson_camera.msg import twovids
from motor_control.msg import motor_cmd
from motor_odometry.msg import odometry


class OdometryPublisherNode:
    def __init__(self):
        self.initialized = False
        rospy.loginfo("Initializing odometry node...")
        self.start_odometry = False
        
        # Construct subscriber
        self.pub_odom = rospy.Publisher(
            "/odometry",
            message_type_odem,
            self.motor_cb,
            buff_size=2**24,
            queue_size=10
        )
        self.gain, self.trim = self.load_param()

        self.first_image_received = False
        self.initialized = True
        rospy.loginfo("odem node initialized!")



    def odometry(self,xticks, yticks):
        
        alpha = 2 * np.pi / self.encoder_resolution 



        return xticks, yticks, position

if __name__ == "__main__":
    # Initialize the node
    rospy.init_node('odometry_node', anonymous=True, xmlrpc_port=45102, tcpros_port=45103)
    odometry_node = MotorSubscriberNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down odometry node.")