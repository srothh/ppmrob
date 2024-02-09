#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from tello import Tello
from action import LaunchAction
from sensor import ImageSensor



def drone_node():
    # Initialize the ROS node
    rospy.init_node('drone', anonymous=True)
    rospy.loginfo("starting drone node")

    # init tello driver
    drone = Tello()
    launch_server = LaunchAction('launch', drone)
    rospy.loginfo("launch server created")

    img = ImageSensor(drone)
    rospy.Timer(rospy.Duration(0.5), img.publish)
    rospy.loginfo("image publisher started")

    rospy.spin()

if __name__ == '__main__':
    try:
        drone_node()
    except rospy.ROSInterruptException:
        pass
