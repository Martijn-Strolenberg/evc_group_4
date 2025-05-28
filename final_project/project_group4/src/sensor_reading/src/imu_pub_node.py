#!/usr/bin/env python2

from imuDriver import mpu6050
import rospy
from sensor_msgs.msg import Imu
from std_srvs.srv import Empty, EmptyResponse

class IMUReaderNode():
    def __init__(self, node_name, freq_divider, ang_vel_offset, accel_offset, i2c_bus, device_address):
        # Node Init
        self.initialized = False
        rospy.loginfo("Initializing IMU reader node...")
        self.node_name = node_name
        rospy.init_node(self.node_name, anonymous=True)
        self.DMP_freq = 40
        self.g = 9.80665
        self._freq_divider = freq_divider
        self._gyro_offset = ang_vel_offset
        self._accel_offset = accel_offset
        self._bus = i2c_bus
        self._address = device_address
        self._offset_init = False
        rospy.loginfo("===============IMU Node Init Val===============")
        rospy.loginfo("Op Rate: %.2f Hz" % (float(self.DMP_freq) / float(self._freq_divider)))
        rospy.loginfo("Acceleration Offset: X:%.2f, Y: %.2f, Z: %.2f m/s^2" % tuple(self._accel_offset))
        rospy.loginfo("Gyro Offset X:%.2f, Y: %.2f, Z: %.2f degrees/s" % tuple(self._gyro_offset))
        rospy.loginfo("===============END of IMU Init Val===============")
        
        # IMU Initialization
        self._sensor = self._find_sensor()
        if not self._sensor:
            rospy.logerr("No MPU6050 device found\n")
            exit(1)
        # ---
        rospy.loginfo("===============Performing Initial Testing!===============")
        accel_dmp = self._sensor.get_accel_data()
        rospy.loginfo("Acceleration: X: {:.2f}, Y: {:.2f}, Z: {:.2f} m/s^2".format(accel_dmp['x'], accel_dmp['y'], accel_dmp['z']))
        gyro_data = self._sensor.get_gyro_data()
        rospy.loginfo("Gyro X: {:.2f}, Y: {:.2f}, Z: {:.2f} degrees/s".format(gyro_data['x'], gyro_data['y'], gyro_data['z']))
        rospy.loginfo("===============IMU Initialization Complete===============")

        # ROS Pubsub initialization
        self.pub = rospy.Publisher('/imu/data_raw', Imu, queue_size=10)
        if self._offset_init == False:
            self.zero_sensor()
            self._offset_init = True

        if self._freq_divider > 0:
            self.timer = rospy.Timer(rospy.Duration.from_sec((float(self._freq_divider) / float(self.DMP_freq))), self.publish_data)
        else:
            rospy.logwarn("Frequency divider is zero or invalid. Defaulting to 1 Hz.")
            self.timer = rospy.Timer(rospy.Duration(1.0), self.publish_data)  # Default 1 Hz frequency
        rospy.loginfo("IMU reader node initialized!")

    def _find_sensor(self):
        conn = "[bus:{bus}](0x{address:02X})".format(bus=self._bus, address=self._address)
        rospy.loginfo("Trying to open device on connector {}".format(conn))

        # Overwrite Adafruit default device ID
        try:
            sensor = mpu6050(self._address, bus=self._bus)
            rospy.loginfo("Device found on connector {}".format(conn))
            return sensor
        except Exception:
            rospy.logwarn("No devices found on connector {}, but the bus exists".format(conn))
            return None

    def publish_data(self, event):
        # Message Blank
        msg = Imu()
        
        try:
            # Time stamp the message
            msg.header.stamp = rospy.Time.now()

            # Get DMP data (acceleration and gyro)
            acc_data_dmp = self._sensor.get_accel_data()
            gyro_data = self._sensor.get_gyro_data()

            # Populate the message
            msg.orientation.x = msg.orientation.y = msg.orientation.z = msg.orientation.w = 0
            msg.orientation_covariance = [0.0 for _ in range(len(msg.orientation_covariance))]
            msg.orientation_covariance[0] = -1

            msg.angular_velocity.x = gyro_data['x'] * 250 / 2**15 - self._gyro_offset[0]
            msg.angular_velocity.y = gyro_data['y'] * 250 / 2**15 - self._gyro_offset[1]
            msg.angular_velocity.z = gyro_data['z'] * 250 / 2**15 - self._gyro_offset[2]
            msg.angular_velocity_covariance = [0.0 for _ in range(len(msg.angular_velocity_covariance))]

            msg.linear_acceleration.x = acc_data_dmp['x'] * 2 * self.g / 2**15 - self._accel_offset[0]
            msg.linear_acceleration.y = acc_data_dmp['y'] * 2 * self.g / 2**15 - self._accel_offset[1]
            msg.linear_acceleration.z = acc_data_dmp['z'] * 2 * self.g / 2**15 - self._accel_offset[2]
            msg.linear_acceleration_covariance = [0.0 for _ in range(len(msg.linear_acceleration_covariance))]

            # Publish the message
            self.pub.publish(msg)

        except Exception as IMUCommLoss:
            rospy.logwarn("IMU Comm Loss: {}".format(IMUCommLoss))
            pass


    def zero_sensor(self):
        rospy.loginfo("zero_sensor service called.")
        # acc_data = self._sensor.get_acceleration()
        acc_data = self._sensor.get_accel_data()
        self._accel_offset = [acc_data['x'] * 2 * self.g / 2**15,
                              acc_data['y'] * 2 * self.g / 2**15,
                              acc_data['z'] * 2 * self.g / 2**15]

        gyro_data = self._sensor.get_gyro_data()
        self._gyro_offset = [gyro_data['x'] * 250 / 2**15,
                             gyro_data['y'] * 250 / 2**15,
                             gyro_data['z'] * 250 / 2**15]

        rospy.loginfo("IMU zeroed with ACC: X:{:.2f}, Y: {:.2f}, Z: {:.2f} m/s^2".format(self._accel_offset[0], self._accel_offset[1], self._accel_offset[2]))
        rospy.loginfo("IMU zeroed with Gyro X:{:.2f}, Y: {:.2f}, Z: {:.2f} degrees/s".format(self._gyro_offset[0], self._gyro_offset[1], self._gyro_offset[2]))
        return None


if __name__ == '__main__':
    node = IMUReaderNode(node_name = "imu_node", freq_divider = 8, ang_vel_offset = [0, 0, 0], 
                         accel_offset = [0, 0, 0], i2c_bus = 1, device_address = 0x68)
    rospy.spin()

