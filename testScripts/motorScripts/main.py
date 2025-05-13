#!/usr/bin/env python2

from motorDriver import *
from time import sleep, time
import sys
import tty
import termios

# ==== Robot Geometry ====
WHEEL_CIRC = 414.69           # mm
TICK_DEG = 2.6
SINGLE_TICK_MM = WHEEL_CIRC * (TICK_DEG / 360)  # mm per encoder tick
WHEEL_BASE = 163.0            # mm (distance between wheels, adjust as needed)

# Movement goals
TARGET_MOVE_MM = 100.0        # 10 cm
TARGET_ROTATE_DEG = 10.0

# Motor speeds
SPEED = 0.3
TURN_SPEED = 0.25

# Encoders
ENC1 = 18
ENC2 = 19

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

def print_controls():
    print("Control the robot:")
    print("  f = move forward 10 cm")
    print("  b = move backward 10 cm")
    print("  l = rotate left 10 deg")
    print("  r = rotate right 10 deg")
    print("  s = stop")
    print("  q = quit")

def reset_encoders(enc1, enc2):
    enc1._ticks = 0
    enc2._ticks = 0

def move_distance(mm, motor, enc1, enc2, speed):
    """Drives straight until a certain distance in mm is reached."""
    reset_encoders(enc1, enc2)
    motor.set_wheels_speed(speed, speed if mm > 0 else -speed)
    target_ticks = abs(mm / SINGLE_TICK_MM)

    while True:
        ticks_avg = (abs(enc1._ticks) + abs(enc2._ticks)) / 2.0
        if ticks_avg >= target_ticks:
            break
        sleep(0.01)

    motor.set_wheels_speed(0, 0)

def rotate_degrees(deg, motor, enc1, enc2, speed):
    """Rotates the robot in place by a certain number of degrees."""
    reset_encoders(enc1, enc2)

    arc_length = (WHEEL_BASE * 3.1416) * (deg / 360.0)  # mm of travel per wheel
    target_ticks = abs(arc_length / SINGLE_TICK_MM)

    # Left turn: left wheel back, right wheel forward
    if deg > 0:
        motor.set_wheels_speed(-speed, speed)
    else:
        motor.set_wheels_speed(speed, -speed)

    while True:
        ticks_avg = (abs(enc1._ticks) + abs(enc2._ticks)) / 2.0
        if ticks_avg >= target_ticks:
            break
        sleep(0.01)

    motor.set_wheels_speed(0, 0)

# === Main Execution ===
try:
    motor = DaguWheelsDriver()
    enc1 = WheelEncoderDriver(ENC1)
    enc2 = WheelEncoderDriver(ENC2)

    print_controls()

    while True:
        key = getch()

        if key == 'f':
            print("Moving forward 10 cm")
            move_distance(TARGET_MOVE_MM, motor, enc1, enc2, SPEED)
        elif key == 'b':
            print("Moving backward 10 cm")
            move_distance(-TARGET_MOVE_MM, motor, enc1, enc2, SPEED)
        elif key == 'l':
            print("Rotating left 10 deg")
            rotate_degrees(TARGET_ROTATE_DEG, motor, enc1, enc2, TURN_SPEED)
        elif key == 'r':
            print("Rotating right 10 deg")
            rotate_degrees(-TARGET_ROTATE_DEG, motor, enc1, enc2, TURN_SPEED)
        elif key == 's':
            motor.set_wheels_speed(0, 0)
            print("Stopped")
        elif key == 'q':
            print("Quitting...")
            break
        else:
            print("Unknown command:", key)

except KeyboardInterrupt:
    print("\nInterrupted by user.")
finally:
    motor.set_wheels_speed(0, 0)
    motor.close()
    print("Motors safely shut down.")
