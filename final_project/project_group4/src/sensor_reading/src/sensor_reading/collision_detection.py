#!/usr/bin/env python2

import rospy
from std_msgs.msg import Float64
from sensor_reading.srv import CollisionDetection
from std_msgs.msg import Bool

class CollisionDetectionNode:
 
    def __init__(self):
        self.initialized = False
        rospy.loginfo("Initializing Collision Detection node...")

        # Make a subscriber to ToF sensor
        self.sub_tof = rospy.Subscriber(
            "/tof",
            Float64,
            self.set_tof_cb,
            buff_size=2**24,
            queue_size=10
        )

        # Make a publisher to detect and set the collision detection to true
        self.pub_collision_detection = rospy.Publisher(
            '/collision_detection',
            Bool,
            queue_size=10
        )

        self.initialized = True
        self.prev_mesg = -1
        self.curr_mesg = 0
        self.state = 0
        self.collision_dist = 100.0
        self.collision_state = False
        rospy.loginfo("Collision Detection node initialized!")


    
    def set_tof_cb(self, data):
        self.tof = data.data
        #rospy.loginfo("TOF distance: %f", self.tof)
        self.state_tof()

    def state_tof(self):
        if self.tof <= self.collision_dist and not self.collision_state:
            rospy.loginfo("Collision detected!")
            self.set_collision_detection(True)
            self.collision_state = True

        elif self.tof > self.collision_dist and self.collision_state:
            rospy.loginfo("No collision.")
            # self.set_collision_detection(False)
            self.collision_state = False

    def set_collision_detection(self, collision):
        # Publish the collision detection status
        self.pub_collision_detection.publish(Bool(data=collision))
        rospy.loginfo("Published collision state: %s", collision)
    
    def service_tof(self,collision):
        rospy.wait_for_service('collision_detection')
        try:
            proxy = rospy.ServiceProxy('collision_detection', CollisionDetection)
            resp = proxy(collision)
            print("Detected Wall:", resp.success)
        except rospy.ServiceException as e:
            print("Service call failed:", e)
        
if __name__ == "__main__":
    # Initialize the node
    rospy.init_node('collision_detection_node')
    collision_detection_node = CollisionDetectionNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down collision detection node.")