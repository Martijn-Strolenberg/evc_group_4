#!/usr/bin/env python2
import rospy
import numpy as np
import cv2
from nav_msgs.msg import Odometry
import tf

class TrackMapper:
    def __init__(self):
        self.mapping = True
        self.loop_detected = False
        self.odom_data = []

        self.start_pose = None
        self.has_moved_enough = False
        self.min_total_distance = 1.0  # meters
        self.total_distance_traveled = 0.0
        self.prev_position = None

        # Visualization setup
        self.window_name = "Track Mapping"
        self.img_size = 800
        self.scale = 50.0  # 1 meter = 50 pixels
        self.center = (self.img_size // 2, self.img_size // 2)
        self.image = np.zeros((self.img_size, self.img_size, 3), dtype=np.uint8)

        # Subscribe to Odometry
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback, queue_size=10)

        rospy.loginfo("TrackMapper node initialized and waiting for odometry...")

        # Wait for odometry to begin
        rospy.sleep(0.5)
        while not self.odom_data and not rospy.is_shutdown():
            rospy.sleep(0.1)
        self.start_pose = self.odom_data[-1]
        self.prev_position = (self.start_pose[0], self.start_pose[1])
        rospy.loginfo("Received initial odometry. Starting visualization...")

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        self.mapping_loop()

    def world_to_image(self, x, y):
        ix = int(self.center[0] + x * self.scale)
        iy = int(self.center[1] - y * self.scale)
        return ix, iy

    def odom_callback(self, msg):
        if not self.mapping:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        yaw = tf.transformations.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w])[2]

        self.odom_data.append((x, y, yaw))

        # Update distance traveled
        if self.prev_position is not None:
            dx = x - self.prev_position[0]
            dy = y - self.prev_position[1]
            step_dist = np.sqrt(dx**2 + dy**2)
            self.total_distance_traveled += step_dist
        self.prev_position = (x, y)

        if self.total_distance_traveled >= self.min_total_distance:
            self.has_moved_enough = True

        # Draw a white dot for the robots position
        px, py = self.world_to_image(x, y)
        if 0 <= px < self.img_size and 0 <= py < self.img_size:
            cv2.circle(self.image, (px, py), 1, (255, 255, 255), -1)

        # Check for loop closure
        if self.start_pose and self.has_moved_enough:
            dx = x - self.start_pose[0]
            dy = y - self.start_pose[1]
            d = np.sqrt(dx**2 + dy**2)
            dyaw = abs(yaw - self.start_pose[2])
            if d < 0.3 and dyaw < 0.2:
                rospy.loginfo("Loop detected.")
                self.loop_detected = True
                self.mapping = False

    def mapping_loop(self):
        rate = rospy.Rate(10)

        while self.mapping and not rospy.is_shutdown():
            # Create a copy for displaying (prevent accumulation of text)
            display_img = self.image.copy()

            # Add overlay text on copy
            cv2.putText(display_img, "Distance: {:.2f} m".format(self.total_distance_traveled),
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2)
            cv2.putText(display_img, "Mapping..." if self.mapping else "Stopped",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2)

            cv2.imshow(self.window_name, display_img)
            key = cv2.waitKey(1)
            if key == 27:
                rospy.loginfo("Mapping manually stopped by user.")
                self.mapping = False
            rate.sleep()

        # Final visualization and shutdown
        cv2.imshow(self.window_name, self.image)
        cv2.waitKey(1000)
        cv2.destroyWindow(self.window_name)
        rospy.loginfo("Mapping finished. Loop detected: {}".format(self.loop_detected))

if __name__ == "__main__":
    rospy.init_node("track_mapper_node")
    TrackMapper()
