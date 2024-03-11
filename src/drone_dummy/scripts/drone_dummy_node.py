#!/usr/bin/env python3


import sys
print('sys.path:', sys.path)


import rospy
from std_msgs.msg import String
from action import LaunchAction
from action import MoveAction
from action import EmergencyAction
from action import CommandAction
from sensor import ImageSensor
from sensor import TwistSensor
from sensor import BatterySensor
import common.config.defaults


def drone_node():

    datadir = "/home/lazafi/labor/mobrob-2023/src/ppmrob2/repo/src/drone_dummy/scripts/data"

    # Initialize the ROS node
    rospy.init_node('drone', anonymous=True)
    rospy.loginfo("starting drone node")

    drone = None

    emergency_server = EmergencyAction('emergency', drone)
    rospy.loginfo("emergency server created")


    launch_server = LaunchAction('launch', drone)
    rospy.loginfo("launch server created")

    launch_server = MoveAction('move', drone)
    rospy.loginfo("move server created")

    launch_server = CommandAction('command', drone)
    rospy.loginfo("command server created")

    img = ImageSensor(datadir+'/camera')
    rospy.Timer(rospy.Duration(0.5), img.publish)
    rospy.loginfo("image publisher started")

    twist = TwistSensor(datadir+'/telemetry')
    rospy.Timer(rospy.Duration(0.1), twist.publish)
    rospy.loginfo("twist publisher started")

    battery = BatterySensor()
    rospy.Timer(rospy.Duration(0.1), battery.publish)
    rospy.loginfo("battery publisher started")


    rospy.loginfo("drone_node started")
    rospy.spin()

if __name__ == '__main__':
    try:
        drone_node()
    except rospy.ROSInterruptException:
        drone.terminate()
        rospy.loginfo("drone node terminated")
        pass
