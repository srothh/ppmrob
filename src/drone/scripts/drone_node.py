#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from tello import Tello
from action import LaunchAction
from action import MoveAction
from action import EmergencyAction
from action import CommandAction
from sensor import ImageSensor
from sensor import TwistSensor
from sensor import BatterySensor
from util import StateCsvLogger
import common.config.defaults
import datetime

def drone_node(drone):
    # Initialize the ROS node
    rospy.init_node("drone", anonymous=True)
    rospy.loginfo("starting drone node")

    drone.connect()

    emergency_server = EmergencyAction(
        common.config.defaults.drone_emergency_action_name, drone
    )
    rospy.loginfo("emergency server created")

    launch_server = LaunchAction(common.config.defaults.drone_launch_action_name, drone)
    rospy.loginfo("launch server created")

    move_server = MoveAction(common.config.defaults.drone_move_action_name, drone)
    rospy.loginfo("move server created")

    command_server = CommandAction(
        common.config.defaults.drone_command_action_name, drone
    )
    rospy.loginfo("command server created")

    img = ImageSensor(
        drone, topic=common.config.defaults.drone_image_sensor_publish_topic_name
    )
    rospy.Timer(
        rospy.Duration(common.config.defaults.drone_image_sensor_publish_delay),
        img.publish,
    )
    rospy.loginfo("image publisher started")

    twist = TwistSensor(
        topic=common.config.defaults.drone_twist_sensor_publish_topic_name
    )
    drone.registerStateHandler(twist.publish)

    battery = BatterySensor(
        topic=common.config.defaults.drone_battery_sensor_publish_topic_name
    )
    drone.registerStateHandler(battery.publish)

    # append timestamp to filename

    writer = StateCsvLogger("./state-log-" +  datetime.datetime.now().strftime("%Y%m%d%H%M%S") + ".cvs")
    drone.registerStateHandler(writer.log)

    # image recorder
    #recorder = ImageRecorder(topic=common.config.defaults.drone_image_sensor_publish_topic_name, dir="./images-" +  datetime.datetime.now().strftime("%Y%m%d%H%M%S"))

    # keepalive
    rospy.Timer(rospy.Duration(10), lambda x: drone.keep_alive())

    rospy.loginfo("drone_node started")
    rospy.spin()


if __name__ == "__main__":
    drone = None
    try:
        # init tello driver
        drone = Tello(command_timeout=common.config.defaults.drone_command_timeout)
        drone_node(drone)
    except rospy.ROSInterruptException:
        drone.terminate()
        rospy.loginfo("drone node terminated")
        pass
