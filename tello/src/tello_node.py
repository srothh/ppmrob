#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

from action.launch_action_server import LaunchActionServer
from action.rotate_action_server import RotateActionServer
from action.x_action_server import XActionServer
from action.y_action_server import YActionServer
from action.z_action_server import ZActionServer
from action.command_action_server import CommandActionServer
from sensor.battery_publisher import BatteryPublisher
from sensor.telemetry_sensor import TelemetrySensor

from djitellopy import Tello

def tello_node():
    # Initialize the ROS node
    rospy.init_node('tello', anonymous=True)
    print("starting tello node")
    # init tello driver
    drone = Tello()
    # start drone
    drone.connect()
    print(drone.query_sdk_version())
    print(drone.query_active())

    # start action servers
    command = CommandActionServer('command', drone)
    launch = LaunchActionServer('launch', drone)
    rotate = RotateActionServer('rotate', drone)
    x = XActionServer('x', drone)
    y = YActionServer('y', drone)
    z = ZActionServer('z', drone)

    # start sensor publishers
    tel = TelemetrySensor(drone)
    rospy.Timer(rospy.Duration(1.0), tel.publish)

    rospy.spin()
    
if __name__ == '__main__':
    try:
        tello_node()
    except rospy.ROSInterruptException:
        pass
