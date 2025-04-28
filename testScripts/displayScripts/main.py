#!/usr/bin/env python2

import Adafruit_SSD1306
from PIL import Image, ImageDraw, ImageFont
from time import sleep

# Initialize the display (I2C address: 0x3C)
disp = Adafruit_SSD1306.SSD1306_128_64(rst=None, i2c_bus=1, i2c_address=0x3C)

# Initialize the library
disp.begin()

# Clear the display
disp.clear()
disp.display()

# Create an image for drawing
width = disp.width
height = disp.height
image = Image.new('1', (width, height))

# Get a drawing object to draw on the image
draw = ImageDraw.Draw(image)

# Load a font
font = ImageFont.load_default()

# Draw static text
draw.rectangle((0, 0, width, height), outline=0, fill=0)  # Clear the image
draw.text((10, 5), "Battery: 75%", font=font, fill=255)
draw.text((10, 20), "Connection: OK", font=font, fill=255)
draw.text((10, 35), "ToF: Active", font=font, fill=255)

# Display the image
disp.image(image)
disp.display()

# Refresh every 5 seconds
while True:
    sleep(5)

