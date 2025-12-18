#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int64

def callback(msg):
    # We need to define callback
    pass

def node():
    # A node is a subscriber initialization
    rospy.init_node("subscriber")
    rospy.Subscriber("topic", Int64, callback)

    rospy.spin()
    
if __name__ == "__main__":
    node()
