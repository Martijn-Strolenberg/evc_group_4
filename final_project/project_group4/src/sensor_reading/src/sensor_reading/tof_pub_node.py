#!/usr/bin/env python2

import rospy
import numpy as np
from sensor_reading.drivers.tofDriver import VL53L0X
from std_msgs.msg import Float64
from collections import deque


class TofPublisherNode:
    def __init__(self):
        self.initialized = False
        rospy.loginfo("Initializing tof publisher node...")
        self.window_size          = rospy.get_param("~window_size", 5)      # samples
        self.max_valid_distance   = rospy.get_param("~max_valid_distance", 1200.0)  # mm
        # Exponential Moving Average (EMA) filter parameters
        self.use_ema              = rospy.get_param("~use_ema", False)
        self.ema_alpha            = rospy.get_param("~ema_alpha", 0.2)      # 0 < alfa < 1

        self.buffer = deque(maxlen=self.window_size)  # for SMA
        self.ema    = None # for EMA (doesn't require a buffer!)
        rospy.loginfo("Starting TOF publisher (window=%d, max=%.1f mm, EMA=%s)", self.window_size, self.max_valid_distance, self.use_ema)

        # Init Publisher
        self.pub_tof = rospy.Publisher(
            "/tof",
            Float64,
            queue_size=10
        )
        self.sensor = VL53L0X()
        rospy.loginfo("VL53L0X ToF sensor initialized.")

    def filter(self, raw_data):
        """Return filtered distance or None if measurement is rejected."""
        # remove invalid values (values that are larger than 1200 [mm])
        if raw_data <= 0.0 or raw_data > self.max_valid_distance:
            rospy.logdebug("Rejected raw value: %.1f mm", raw_data)
            return None
        
        # Exponential Moving Average (EMA) filter
        if self.use_ema:                     
            self.ema = raw_data if self.ema is None else \
                       (self.ema_alpha * raw_data + (1.0 - self.ema_alpha) * self.ema)
            return self.ema

        # Simple Moving Average (SMA) filter
        else:                                
            self.buffer.append(raw_data)
            return float(np.mean(self.buffer))


    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            try:
                raw_distance = self.sensor.read_distance()
                filtered     = self.filter(raw_distance)
                if filtered is not None:
                    self.pub_tof.publish(Float64(filtered))
            except rospy.ROSInterruptException:
                break
            except Exception as err:          # keep node alive on non-critical errors
                rospy.logwarn("TOF read/publish error: %s", err)
            rate.sleep()
    

# Main function run the node
if __name__ == "__main__":
    # Initialize the node
    rospy.init_node('tof_pub_node')
    TofPublisherNode().run()
