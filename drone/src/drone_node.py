#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from tello import Tello
from action import LaunchAction
from action import MoveAction
from action import EmergencyAction
from action import CommandAction
from sensor import ImageSensor



def drone_node(tello):
    # Initialize the ROS node
    rospy.init_node('drone', anonymous=True)
    rospy.loginfo("starting drone node")


    drone.connect()

    emergency_server = EmergencyAction('emergency', drone)
    rospy.loginfo("emergency server created")


    launch_server = LaunchAction('launch', drone)
    rospy.loginfo("launch server created")

    launch_server = MoveAction('move', drone)
    rospy.loginfo("move server created")

    launch_server = CommandAction('command', drone)
    rospy.loginfo("command server created")

    img = ImageSensor(drone)
    rospy.Timer(rospy.Duration(0.5), img.publish)
    rospy.loginfo("image publisher started")

    rospy.spin()

if __name__ == '__main__':
    drone = None
    try:
        # init tello driver
        drone = Tello()
        drone_node(drone)
    except rospy.ROSInterruptException:
        drone.terminate()
        rospy.loginfo("drone node terminated")
        pass
