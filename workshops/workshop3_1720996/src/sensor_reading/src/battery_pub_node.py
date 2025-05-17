#!/usr/bin/env python2
import serial
import time
import re
# INFO_COMMAND = b"??"
# SHUTDOWN_COMMAND = b"QQ
import rospy
from std_msgs.msg import Float64
import time


class Battery_Node:
    def __init__(self, node_name):
        self.initialized = False
        self.node_name = node_name
        rospy.loginfo("Initializing ROS node...")
        rospy.init_node(self.node_name, anonymous=True)

        # Construct publishers
        self.battery_pub = rospy.Publisher(
            "/Battery",
            Float64,
            #Change buff size and queue size accordingly
            queue_size=1,
        )

        self.initialized = True
        rospy.loginfo("Node initialized!")
  
    def pub_info_battery(self, serial_port="/dev/ttyACM0", baud_rate=9600):
        try:
            self.ser = serial.Serial(port=serial_port, baudrate=baud_rate, timeout=10)
            rospy.loginfo("Connected to Duckiebattery on {} at {} baud.".format(serial_port, baud_rate))
        except serial.SerialException as e:
            rospy.logerr("Serial connection failed: {}".format(e))
            return
        # Command to be sent
        command = b"?"

        while not rospy.is_shutdown():
            try:
                # Send the shutdown command
                self.ser.write(command)
                rospy.loginfo("Info command sent to Duckiebattery.")

                # Optional: Read response from Duckiebattery
                response = self.ser.readline().decode("utf-8").strip()
                if response:
                    rospy.loginfo("Duckiebattery response: {}".format(response))
                # Extract all numbers (int or float)
                numbers = re.findall(r'\d+(?:\.\d+)?', response)
                int_numbers = [int(num) for num in numbers]
                # print(int_numbers)  # Output: ['11.7', '84']
                # print(type(int_numbers))
                # Close the serial connection
                msg = float(int_numbers[0]) 
                # Close the serial connection
                rospy.loginfo("Duckiebattery percentage: {}".format(msg))
                self.battery_pub.publish(msg)

            except serial.SerialException as e:
                print("Serial exception occurred: {}".format(e))
            except Exception as e:
                print("An error occurred while fetching info: {}".format(e))
        
    def cleanup(self):
        self.ser.close()
        

if __name__ == "__main__":
    node_name = "battery_publisher_node"
    camera_pub = None
    try:
        battery_pub = Battery_Node(node_name)
        battery_pub.pub_info_battery()
    except rospy.ROSInterruptException:
        pass
    finally:
        battery_pub.cleanup()

    