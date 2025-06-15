#!/usr/bin/env python3

import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
from pyzbar import pyzbar
from std_msgs.msg import Float64, UInt8

class QRDetectionNode:
    def __init__(self):
        self.initialized = False
        self.node_name = "qr_recognition"
        rospy.loginfo("Initializing QR detection node...")

        self.bridge = CvBridge()

        # Subscriber to combined image topic
        self.sub_image = rospy.Subscriber(
            "/camera/image_proc",
            CompressedImage,
            self.qr_recognition_cb,
            buff_size=2**24,
            queue_size=1
        )

        # Init Publisher
        self.pub_find = rospy.Publisher(
            "/find_object",
            UInt8,
            queue_size=10
        )

        self.first_image_received = False
        self.initialized = True

        # Check if qr code has already seen and gotten the info
        self.find_obj = -1
        self.prev_obj = -1

        # message variables to not send message many times
        self.curr_msg = 0
        self.prev_msg = 0
        rospy.loginfo("QR detection node initialized.")

    def detect_qr_codes(self, frame):
        qr_data_return = ""
        decoded_objects = pyzbar.decode(frame)

        for obj in decoded_objects:
            points = obj.polygon
            if not points:
                continue

            try:
                # Get convex hull if more than 4 points
                if len(points) > 4:
                    hull = cv2.convexHull(
                        np.array([[p.x, p.y] for p in points], dtype=np.float32)
                    )
                    points = [tuple(point[0]) for point in hull]
                else:
                    points = [(p.x, p.y) for p in points]

                # Draw lines
                for i in range(len(points)):
                    pt1 = (int(points[i][0]), int(points[i][1]))
                    pt2 = (int(points[(i + 1) % len(points)][0]), int(points[(i + 1) % len(points)][1]))
                    cv2.line(frame, pt1, pt2, (0, 255, 0), 2)

                # Decode text
                qr_data = obj.data.decode("utf-8")
                cv2.putText(frame, str(qr_data), (int(points[0][0]), int(points[0][1]) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

                rospy.loginfo("Detected QR code: {}".format(qr_data))

                try:
                    if qr_data_return == "":
                        qr_data_return = qr_data
                    
                except ValueError:
                    rospy.logwarn("QR data not an integer: {}".format(qr_data))
            except Exception as e:
                rospy.logwarn("QR drawing error: {}".format(e))

        return qr_data_return

    def qr_recognition_cb(self, data):
        if not self.initialized:
            return
        
        if not self.first_image_received:
            self.first_image_received = True
            rospy.loginfo("Receiving images...")

        try:
            # Decode images
            undist_image = cv2.imdecode(np.frombuffer(data.data, np.uint8), cv2.IMREAD_COLOR)

            if undist_image is None:
                rospy.logwarn("Image decoding failed.")
                return

            # Copy for annotation
            qr_frame = undist_image.copy()

            # Detect QR and annotate
            qr_obj = self.detect_qr_codes(qr_frame)

            # Type of object detected (0: unknown, 1: blue, 2: orange, 3: green, 4: red)
            # Save object to be found:
            if qr_obj == "blue":
                self.find_obj = 1
            elif qr_obj == "orange":
                self.find_obj = 2
            elif qr_obj == "green":
                self.find_obj = 3
            elif qr_obj == "red":
                self.find_obj = 4
            
            # If a different colour is recieved, a new message has occured
            if self.find_obj != self.prev_obj:
                self.curr_msg += 1 
                self.prev_obj = self.find_obj
                rospy.loginfo("Finding {colour_obj} object".format(colour_obj = qr_obj))

            # Show
            cv2.imshow("QR code detection", qr_frame)
            
            # Publishing the MSG
            msg = UInt8()
            if self.curr_msg != self.prev_msg: # New QR recieved
                # Send message  
                msg.data = self.find_obj
                rospy.loginfo("Publishing find object message: %d", msg.data)
                
                #Updated message
                self.prev_msg = self.curr_msg
                self.pub_find.publish(msg)
            cv2.waitKey(1)

        except CvBridgeError as err:
            rospy.logerr("CvBridge error: {}".format(err))
        except Exception as e:
            rospy.logerr("Unhandled exception: {}".format(e))
        
    def cleanup(self):
        cv2.destroyAllWindows()

# Main
if __name__ == "__main__":
    rospy.init_node('qr_node', anonymous=False, xmlrpc_port=45100, tcpros_port=45101)
    qr_node = QRDetectionNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down QR node.")
    finally:
        qr_node.cleanup()