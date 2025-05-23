#!/usr/bin/env python

import rospy
from sensor_reading.srv import BatteryInfo

def battery_monitor_loop():
  rospy.wait_for_service("battery_info") # this is blocking until the service "battery_info" is available
  proxy = rospy.ServiceProxy("battery_info", BatteryInfo)
  rospy.loginfo("Battery info service is started.")

  rate = rospy.Rate(1.0 / 30.0)  # Every 30 seconds
  while not rospy.is_shutdown():
    try:
      response = proxy()
      rospy.loginfo("Battery Percentage: {}".format(response.percentage))
    except rospy.ServiceException as e:
      rospy.logwarn("Failed to get battery status: %s", e)
    rate.sleep()

if __name__ == "__main__":
  rospy.init_node("battery_monitor")
  battery_monitor_loop()