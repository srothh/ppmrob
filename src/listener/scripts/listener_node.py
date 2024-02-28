#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(f"Received message: {data.data}")

def listener_node():
    # Initialize the ROS node
    rospy.init_node('listener', anonymous=True)

    # Subscribe to the 'chatter' topic and register the callback function
    rospy.Subscriber('chatter', String, callback)

    # Spin to keep the script from exiting
    rospy.spin()

if __name__ == '__main__':
    listener_node()
