#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import Imu

class IMUPrinterNode:
    def __init__(self, node_name):
        self.initialized = False
        self.node_name = node_name
        rospy.loginfo("Initializing IMU printer node...")
        rospy.init_node(self.node_name, anonymous=True)
        
        # Construct subscribers
        self.sub_imu = rospy.Subscriber(
            "/imu/data_raw",
            Imu,
            self.imu_cb,
            buff_size=10000000,
            queue_size=1,
        )

        self.first_imu_received = False
        self.initialized = True
        rospy.loginfo("IMU printer node initialized!")

    def imu_cb(self, data):
        if not self.initialized:
            return
        
        if self.first_imu_received == False:
            self.first_imu_received = True
            rospy.loginfo("IMU printer node captured first IMU measurement from publisher.")
        # Log IMU data
        rospy.loginfo("IMU Data - Orientation: ({x}, {y}, {z}), "
                    "Angular Velocity: ({x1}, {y1}, {z1}), "
                    "Linear Acceleration: ({x2}, {y2}, {z2})".format(
                    x=data.orientation.x, y=data.orientation.y, z=data.orientation.z,
                    x1=data.angular_velocity.x, y1=data.angular_velocity.y, z1=data.angular_velocity.z,
                    x2=data.linear_acceleration.x, y2=data.linear_acceleration.y, z2=data.linear_acceleration.z))
        # Store latest IMU data
        self.latest_imu_data = data

if __name__ == "__main__":
    try:
        camera_node = IMUPrinterNode(node_name = "imu_printer_node")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
