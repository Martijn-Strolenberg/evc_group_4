#!/usr/bin/env python2

from motorDriver import *
from time import sleep

# Motor values
velocity = .1 # Must be contained in [-1,1]

try:
    motor = DaguWheelsDriver()
    motor.set_wheels_speed(left=velocity, right=velocity)
    while True:
        sleep(10)
except KeyboardInterrupt:
    print("\nExiting...")
finally:
    motor.close()
