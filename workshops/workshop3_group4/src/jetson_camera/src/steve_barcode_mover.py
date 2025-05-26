#!/usr/bin/env python2

import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from jetson_camera.msg import twovids
from motor_control.msg import motor_cmd
from pyzbar.pyzbar import decode
import time



class CameraSubscriberNode:
    def __init__(self):
        self.initialized = False
        rospy.loginfo("Initializing camera object detection node...")
        self.bridge = CvBridge()
        
        # Construct subscriber
        self.sub_image = rospy.Subscriber(
            "/camera/image_proc",
            twovids,
            self.image_cb,
            buff_size=2**24,
            queue_size=1
        )

        # Publisher for motor commands (example)
        self.pub_cmd = rospy.Publisher(
            "/motor_control",
            motor_cmd,
            queue_size=10
        )

        self.new_cmd = 0 
        self.move_vel = 0.45
        self.barcodes = []
        self.display_timeout = 0.5
        self.first_image_received = False

        # Map F, to angle 0, R to angle 90, L to angle -90 and B to angle 180
        self.angle_map = {
            'F': 0,
            'R': 90,
            'L': -90,
            'B': 180
        }

        self.initialized = True
        rospy.loginfo("Camera object detection node initialized!")

# <============== Image Processing =================>
    def decode_barcode(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (0, 0), 3)
        sharp = cv2.addWeighted(gray, 1.5, blur, -0.5, 0)

        barcode_image = image.copy()

        for barcode in decode(sharp):  
            found = False
            # Check if the barcode is already in the list and update rectangle and time
            for existing_barcode in self.barcodes:
                if existing_barcode['data'] == barcode.data.decode('utf-8'):
                    existing_barcode['rect'] = barcode.rect
                    existing_barcode['time'] = time.time()
                    found = True
                    break
            if not found:
                # Add new barcode to the list
                self.barcodes.append({
                    'data': barcode.data.decode('utf-8'),
                    'rect': barcode.rect,
                    'time': time.time()
                })
            # parse the string to get the values
            data = barcode.data.decode('utf-8').split(',')
            if len(data) == 3:
                try:
                    # Convert the values to float
                    new_cmd = int(data[0])
                    new_distance = float(data[1])
                    new_angle = self.angle_map[data[2]]
                    if new_cmd != self.new_cmd:
                        self.new_cmd = new_cmd
                        # Send the command to the motor
                        rospy.loginfo("Sending command: {} with distance: {}, angle: {}".format(new_cmd, new_distance, new_angle))
                    self.motor_cmd(self.move_vel, new_distance, new_angle, new_cmd)
                except ValueError:
                    pass


            # Remove barcodes that have not been updated for a certain time
        self.barcodes = [b for b in self.barcodes if time.time() - b['time'] < self.display_timeout]

        #Draw the barcode rectangles + text
        for barcode in self.barcodes:
            cv2.putText(barcode_image, barcode['data'],
                        (barcode['rect'].left, barcode['rect'].top - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return barcode_image




    def image_cb(self, data):
        if not self.initialized:
            return
        
        if not self.first_image_received:
            self.first_image_received = True
            rospy.loginfo("Camera subscriber captured first image from publisher.")
        try:
            # Decode image without CvBridge
            cv_image = cv2.imdecode(np.frombuffer(data.undist_img.data, np.uint8), cv2.IMREAD_COLOR)
            # rospy.loginfo("PubSub delay: {}".format((rospy.Time.now() - data.header.stamp).to_sec()))

            # Decode the barcode
            barcode_image  = self.decode_barcode(cv_image)

            # Show both images
            cv2.imshow("Barcode Decoded Image", barcode_image)
            cv2.waitKey(1)

        except CvBridgeError as err:
            rospy.logerr("Error converting image: {}".format(err))


# <=================== Motor control =========================>
    def motor_cmd(self, velocity, distance, angle, new_cmd):
        # Create a new motor_cmd message
        new_motor_cmd = motor_cmd()
        new_motor_cmd.new_mesg = new_cmd-1
        new_motor_cmd.velocity = velocity
        new_motor_cmd.distance = distance
        new_motor_cmd.angle = np.deg2rad(angle)
        # Publish the command
        self.pub_cmd.publish(new_motor_cmd)


    def cleanup(self):
        cv2.destroyAllWindows()

if __name__ == "__main__":
    # Initialize the node
    rospy.init_node('barcode_detection_node', anonymous=False, xmlrpc_port=45102, tcpros_port=45103)
    camera_node = CameraSubscriberNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down image viewer node.")
    finally:
        camera_node.cleanup()
