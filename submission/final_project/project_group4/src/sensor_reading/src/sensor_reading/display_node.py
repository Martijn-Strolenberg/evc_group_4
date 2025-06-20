#!/usr/bin/env python2

import rospy
import Adafruit_SSD1306
from PIL import Image, ImageDraw, ImageFont
from std_msgs.msg import Float64
from sensor_reading.srv import BatteryInfo


class DisplaySubscriberNode:
 
    def __init__(self):
        self.initialized = False
        rospy.loginfo("Initializing Display node...")

        self.tof = -1
        self.battery_soc = -1

        # Initialize the display (I2C address: 0x3C)
        self.disp = Adafruit_SSD1306.SSD1306_128_64(rst=None, i2c_bus=1, i2c_address=0x3C)

        # Initialize the library
        self.disp.begin()

        # Clear the display
        self.disp.clear()
        self.disp.display()

        # Create an image for drawing
        self.width = self.disp.width
        self.height = self.disp.height
        self.image = Image.new('1', (self.width, self.height))

        # Get a drawing object to draw on the image
        self.draw = ImageDraw.Draw(self.image)

        # Load a font
        self.font = ImageFont.load_default()

        self.sub_tof = rospy.Subscriber(
            "/tof",
            Float64,
            self.set_tof_cb,
            buff_size=2**24,
            queue_size=10
        )
        rospy.loginfo("Waiting for battery_info service ...")
        rospy.wait_for_service("battery_info")
        self.batt_srv = rospy.ServiceProxy("battery_info", BatteryInfo)
        rospy.loginfo("battery_info service found")
        # Poll the service every 30 s
        rospy.Timer(rospy.Duration(30.0), self.query_battery)

        self.initialized = True

        rospy.loginfo("Display node initialized!")

    def query_battery(self, _evt):
        try:
            resp = self.batt_srv()
            self.battery_soc = resp.percentage
            rospy.logdebug("Battery %.1f %%", self.battery_soc)
            self.update_display()
        except rospy.ServiceException as e:
            rospy.logwarn("Battery service call failed: %s", e)
    
    def set_tof_cb(self, msg):
        self.tof = msg.data
        #rospy.loginfo("TOF distance: %f", self.tof)

        # Update the display with the latest data
        if self.initialized:
            self.update_display()
        
    def update_display(self):
        # Draw dynamic text
        self.draw.rectangle((0, 0, self.width, self.height), outline=0, fill=0)  # Clear the image
        self.draw.text((10, 5), "Battery: " + str(self.battery_soc), font=self.font, fill=255)
        self.draw.text((10, 20), "Connection: OK", font=self.font, fill=255)
        self.draw.text((10, 35), "ToF: Distance =" + str(self.tof) , font=self.font, fill=255)

        # Display the image
        self.disp.image(self.image)
        self.disp.display()

if __name__ == "__main__":
    # Initialize the node
    rospy.init_node('display_node')
    display_node = DisplaySubscriberNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down display node.")