from time import sleep
from encoderDriver import *

GPIO_MOTOR_ENCODER_1=18
GPIO_MOTOR_ENCODER_2=19
gpio_pin = 19

driver_1 = WheelEncoderDriver(GPIO_MOTOR_ENCODER_1)
driver_2 = WheelEncoderDriver(GPIO_MOTOR_ENCODER_2)


while True:
    print("Motor Encoder 1: {} \t Motor Encoder 2: {}".format(driver_1._ticks, driver_2._ticks))
    sleep(1)
