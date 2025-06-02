#!/usr/bin/env python2

import re
import rospy
from sensor_reading.srv import BatteryInfo, BatteryInfoResponse
import serial

INFO_COMMAND = b"?"

def handle_battery_info(req, serial_port="/dev/ttyACM0", baud_rate=9600):
    percentage = get_battery_info(serial_port, baud_rate)
    rospy.loginfo("Battery status requested, returning {}".format(percentage))
    return BatteryInfoResponse(percentage)
        
def get_battery_info(serial_port, baud_rate):
    try:
        # Open serial connection to the Duckiebattery
        ser = serial.Serial(port=serial_port, baudrate=baud_rate, timeout=10)

        # Send the info command
        ser.write(INFO_COMMAND)

        # Read response from Duckiebattery
        response = ser.readline().decode("utf-8").strip()
        if response:
            #print("Duckiebattery response: {}".format(response))
            # Extract all numbers (int or float)
            numbers = re.findall(r'\d+(?:\.\d+)?', response)
            float_numbers = [float(num) for num in numbers]
            return float_numbers[0]

        ser.close()

    except serial.SerialException as e:
        print("Serial exception occurred: {}".format(e))
    except Exception as e:
        print("An error occurred while sending the shutdown command: {}".format(e))

if __name__ == "__main__":
  rospy.init_node("battery_monitor_service")
  service = rospy.Service("battery_info", BatteryInfo, handle_battery_info)
  rospy.loginfo("Battery info service is ready.")
  rospy.spin()
