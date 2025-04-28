In order to display anything to the Adafruit SSD1306 Display, alterations need to be made in the Adafruit_GPIO library, so that it supports Jetson GPIOs as well as building the Adafruit SSD1306 repository, since the pip package is not supported for Python 2.7. The library's directory is added to the PYTHONPATH string, seen in the bashrc's line:
export PYTHONPATH=\$PYTHONPATH:/EVC/testScripts/displayScripts/Adafruit_Python_SSD1306
If you change the library's path, be sure to change the command above appropriately, and then run:
source ~/.bashrc
The change made in Adafruit_GPIO is in module GPIO.py, where a new case was added for Jetson Nano:
def get_platform_gpio():
    import Jetson.GPIO as GPIO
    return GPIO
In case you happen to reinstall Adafruit_GPIO, do not forget to make these changes in the libary's location in /home/jetbot/.local/lib/python2.7/site-packages/Adafruit_GPIO/GPIO.py
