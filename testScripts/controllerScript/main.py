#!/usr/bin/env python2

import time
from motorDriver import DaguWheelsDriver
import math

# CONSTANTS
WHEEL_RADIUS = 0.03       # in meters
BASELINE = 0.1            # distance between wheels in meters
GAIN = 1.0
TRIM = 0.0
VELOCITY = 0.5            # linear speed (m/s)
ANGULAR_VELOCITY = 1.0    # rotational speed (rad/s)

def rotate(motor, angle_rad):
    """Rotate the robot by a given angle in radians (positive: left, negative: right)"""
    direction = 1 if angle_rad > 0 else -1
    duration = abs(angle_rad) / ANGULAR_VELOCITY

    left_speed = -direction * (GAIN - TRIM) * VELOCITY
    right_speed = direction * (GAIN + TRIM) * VELOCITY

    print("Rotating for {:.2f} seconds...".format(duration))
    motor.set_wheels_speed(left=left_speed, right=right_speed)
    time.sleep(duration)
    motor.set_wheels_speed(0, 0)

def move_forward(motor, distance):
    """Move the robot straight forward for a given distance (in meters)"""
    duration = distance / VELOCITY

    print("Moving forward for {:.2f} seconds...".format(duration))
    speed = VELOCITY
    motor.set_wheels_speed(
        left=(GAIN - TRIM) * speed,
        right=(GAIN + TRIM) * speed
    )
    time.sleep(duration)
    motor.set_wheels_speed(0, 0)

if __name__ == "__main__":
    motor = DaguWheelsDriver()

    try:
        while True:
            angle_deg = float(raw_input("Enter angle to rotate (degrees, positive=left, negative=right, 0=skip): "))
            distance = float(raw_input("Enter distance to move forward (meters): "))

            angle_rad = math.radians(angle_deg)

            if angle_deg != 0:
                rotate(motor, angle_rad)

            if distance > 0:
                move_forward(motor, distance)

            cont = raw_input("Do you want to enter another command? (y/n): ")
            if cont.lower() != 'y':
                break

    except KeyboardInterrupt:
        print("Interrupted by user.")

    finally:
        motor.set_wheels_speed(0, 0)
        motor.close()
        print("Motor stopped.")

