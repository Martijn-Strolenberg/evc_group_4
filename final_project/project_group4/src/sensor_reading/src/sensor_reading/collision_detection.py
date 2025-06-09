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

        # Make a publisher to

        self.initialized = True
        self.prev_mesg = -1
        self.curr_mesg = 0
        self.state = 0
        self.collision_dist = 100.0
        rospy.loginfo("Collision Detection node initialized!")


    
    def set_tof_cb(self, data):
        self.tof = data.data
        #rospy.loginfo("TOF distance: %f", self.tof)
        self.state_tof()

    def state_tof(self):
        rospy.loginfo("check")
        if self.tof <= self.collision_dist and self.curr_mesg != self.prev_mesg:
            rospy.loginfo("Collision detected!")
            self.service_tof(True)
            self.prev_mesg = self.curr_mesg
        elif self.tof > self.collision_dist and self.prev_mesg == self.curr_mesg:
            rospy.loginfo("no collision")
            self.service_tof(False)
            self.curr_mesg += 1
    
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