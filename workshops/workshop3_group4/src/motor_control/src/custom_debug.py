import rospy

DEBUG_MODE = True

def debug_print(msg):
    if DEBUG_MODE:
        rospy.loginfo(msg)
