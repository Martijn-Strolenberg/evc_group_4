#!/usr/bin/env python2

from motorDriver import *
from time import sleep
import sys
import tty
import termios

def getch():
    """Reads a single character from the terminal."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

# Motor speeds
SPEED = 0.2
LEFT_SPEED = 0.15
RIGHT_SPEED = 0.15

def print_controls():
    print("Control the robot using:")
    print("  f = forward")
    print("  b = backward")
    print("  l = turn left")
    print("  r = turn right")
    print("  s = stop")
    print("  q = quit")

try:
    motor = DaguWheelsDriver()
    print_controls()

    while True:
        key = getch()

        if key == 'f':
            motor.set_wheels_speed(left=SPEED, right=SPEED)
            print("Moving forward")
        elif key == 'b':
            motor.set_wheels_speed(left=-SPEED, right=-SPEED)
            print("Moving backward")
        elif key == 'l':
            motor.set_wheels_speed(left=-LEFT_SPEED, right=RIGHT_SPEED)
            print("Turning left")
        elif key == 'r':
            motor.set_wheels_speed(left=LEFT_SPEED, right=-RIGHT_SPEED)
            print("Turning right")
        elif key == 's':
            motor.set_wheels_speed(left=0, right=0)
            print("Stopped")
        elif key == 'q':
            print("Quitting...")
            break
        else:
            print("Unknown command:", key)

        sleep(1)
        motor.set_wheels_speed(left=0, right=0)


except KeyboardInterrupt:
    print("\nInterrupted by user.")
finally:
    motor.set_wheels_speed(0, 0)
    motor.close()
    print("Motors safely shut down.")
