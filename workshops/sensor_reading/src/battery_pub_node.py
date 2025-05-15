import serial
import time

INFO_COMMAND = b"??"
SHUTDOWN_COMMAND = b"QQ"




if __name__ == "__main__":
    # Adjust the serial port and baud rate as needed
    Info_duckiebattery(serial_port="/dev/ttyACM0", baud_rate=9600)


#!/usr/bin/env python2

import rospy
from std_msgs.msg import Float64
import time


class Node:
    def __init__(self, node_name):
        self.initialized = False
        self.node_name = node_name
        rospy.loginfo("Initializing ROS node...")
        rospy.init_node(self.node_name, anonymous=True)

        # Construct publishers
        self.publisher = rospy.Publisher(
            "/Battery",
            Float64,
            #Change buff size and queue size accordingly
            queue_size=1,
        )

        self.recvSequence = 0
        self.sendSequence = 0

        self.initialized = True
        rospy.loginfo("Node initialized!")
        self.timer = rospy.Timer(rospy.Duration(1.0), self.publish_time)


    def subscriber_cb(self, data):
        if not self.initialized:
            return
        
        self.recvSequence += 1
        rospy.loginfo("Received message ({seqNumber}): {payload}".format(seqNumber=self.recvSequence,payload=data.data))
    
    def publish_time(self, event):
        msg = Float64()
        msg.data = time.time()
        Info_duckiebattery(serial_port="/dev/ttyACM0", baud_rate=9600)

        self.publisher.publish(msg)
        self.sendSequence += 1
        
        rospy.loginfo("Message sent (({seqNumber})): {payload}".format(seqNumber=self.sendSequence,payload=msg))

    
def Info_duckiebattery(serial_port="/dev/ttyACM0", baud_rate=9600):
    """
    Sends a shutdown command to the Duckiebattery via USB serial connection.
    """
    # Command to be sent
    command = b"?"
    try:
        # Open serial connection to the Duckiebattery
        ser = serial.Serial(port=serial_port, baudrate=baud_rate, timeout=10)
        rospy.loginfo("Connected to Duckiebattery on {} at {} baud.".format(serial_port, baud_rate))

        # Send the shutdown command
        ser.write(command)
        rospy.loginfo("Info command sent to Duckiebattery.")

        # Optional: Read response from Duckiebattery
        response = ser.readline().decode("utf-8").strip()
        if response:
            rospy.loginfo("Duckiebattery response: {}".format(response))

        # Close the serial connection
        ser.close()

    except serial.SerialException as e:
        print("Serial exception occurred: {}".format(e))
    except Exception as e:
        print("An error occurred while fetching info: {}".format(e))



if __name__ == "__main__":
    try:
        camera_node = Node(node_name = "template_node")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
