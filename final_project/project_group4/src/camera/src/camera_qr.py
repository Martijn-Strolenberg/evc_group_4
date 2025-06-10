#!/usr/bin/env python2

import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
from jetson_camera.msg import twovids, speed
from pyzbar import pyzbar
from std_msgs.msg import Float64, UInt8
from camera.msg import ObjectDetection
#from motor_control.msg import motor_cmd

class QRDetectionNode:
    def __init__(self):
        self.initialized = False
        self.node_name = "qr_recognition"
        rospy.loginfo("Initializing QR detection node...")

        self.bridge = CvBridge()

        # Subscriber to combined image topic
        self.sub_image = rospy.Subscriber(
            "/",
            CompressedImage,
            self.qr_recognition_cb,
            buff_size=2**24,
            queue_size=1
        )

        # Init Publisher
        self.pub_cmd = rospy.Publisher(
            "/motor_control",
            ObjectDetection,
            queue_size=10
        )

        self.first_image_received = False
        self.initialized = True

        rospy.loginfo("QR detection node initialized.")

    def detect_qr_codes(self, frame):
        qr_data_return = -1
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
                    val = int(qr_data)
                    if qr_data_return == -1 or val < qr_data_return:
                        qr_data_return = val
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
            # Decode both images
            raw_image = cv2.imdecode(np.frombuffer(data.raw_img.data, np.uint8), cv2.IMREAD_COLOR)
            undist_image = cv2.imdecode(np.frombuffer(data.undist_img.data, np.uint8), cv2.IMREAD_COLOR)

            if raw_image is None or undist_image is None:
                rospy.logwarn("Image decoding failed.")
                return

            # Copy for annotation
            qr_frame = undist_image.copy()

            # Detect QR and annotate
            qr_speed = self.detect_qr_codes(qr_frame)

            # Resize if necessary for display
            if raw_image.shape != qr_frame.shape:
                qr_frame = cv2.resize(qr_frame, (raw_image.shape[1], raw_image.shape[0]))

            # Concatenate side by side
            side_by_side = np.hstack((raw_image, qr_frame))

            # Labels
            cv2.putText(side_by_side, "Original", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(side_by_side, "Undistorted + QR", 
                        (raw_image.shape[1] + 10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            # Show
            cv2.imshow("Original vs Undistorted with QR", side_by_side)

            # Publishing the MSG
            msg = ObjectDetection()
            #msg.velocity = 0.4
            #msg.distance = 0
            #msg.angle = np.deg2rad(qr_speed)
            #msg.new_mesg = self.new_cmd
            
            rospy.loginfo("Publishing motor command: v=%.2f\tp=%.2f\ta=%.2f rad", msg.velocity, msg.position, msg.angle)
            self.pub_cmd.publish(msg)

            cv2.waitKey(1)

        except CvBridgeError as err:
            rospy.logerr("CvBridge error: {}".format(err))
        except Exception as e:
            rospy.logerr("Unhandled exception: {}".format(e))
        
    def cleanup(self):
        cv2.destroyAllWindows()

# Main
if __name__ == "__main__":
    rospy.init_node('qr_node', anonymous=True)
    qr_node = QRDetectionNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down QR node.")
    finally:
        qr_node.cleanup()