#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from tello.msg import Battery
from tello.msg import Telemetry

def callback(data):
    rospy.loginfo("Received message: %d %d %d %d" % (data.bat, data.vgx, data.vgy, data.vgz))

def listener_node():
    # Initialize the ROS node
    rospy.init_node('listener', anonymous=True)

    # Subscribe to the 'chatter' topic and register the callback function
    rospy.Subscriber('telemetry', Telemetry, callback)

    # Spin to keep the script from exiting
    rospy.spin()

if __name__ == '__main__':
    listener_node()
