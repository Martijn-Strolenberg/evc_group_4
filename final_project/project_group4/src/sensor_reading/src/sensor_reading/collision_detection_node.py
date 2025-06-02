#!/usr/bin/env python2

import rospy
from std_msgs.msg import Float64
from sensor_reading.srv import CollisionDetection

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
        self.pub_tof

        self.initialized = True
        rospy.loginfo("Collision Detection node initialized!")
        self.state = 0

    
    def set_tof_cb(self, data):
        self.tof = data.data
        rospy.loginfo("TOF distance: %f", self.tof)
    def state_tof(self):
        if self.tof <= 200:
            self.service_tof(True)
        else:
            self.service_tof(False)
    
    def service_tof(self,collision):
        try:
            proxy = rospy.ServiceProxy('col_detect', CollisionDetection)
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