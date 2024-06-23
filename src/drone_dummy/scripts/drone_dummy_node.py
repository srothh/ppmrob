#!/usr/bin/env python3

import rospy
import os
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


    #TODO: localy not set by debugger
    # if rospy.has_param('~basedir'):
    #     datadir = rospy.get_param('~basedir')
    # elif os.environ.get('ROS_WORKSPACE'):
    #     datadir = os.environ.get('ROS_WORKSPACE') + '/src/drone_dummy/data'
    # else:
    #     datadir = 'src/drone_dummy/data'

    # if os.path.exists(dir):
    #     rospy.loginfo("base data directory: " + datadir)
    # else:
    #     raise Exception("directory " + datadir + " does not exist")

    # Initialize the ROS node
    rospy.init_node('drone_dummy', anonymous=True)
    rospy.loginfo("starting drone_dummy node")

    drone = None

    if rospy.has_param('~images'):
        imagesdir = rospy.get_param('~images')
    elif os.environ.get('ROS_WORKSPACE'):
        imagesdir = os.environ.get('ROS_WORKSPACE') + '/src/drone_dummy/data'
    else:
        imagesdir = None

    if rospy.has_param('~statelog'):
        logfile = rospy.get_param('~statelog')
    elif os.environ.get('ROS_WORKSPACE'):
        logfile = os.environ.get('ROS_WORKSPACE') + '/src/drone_dummy/data/telemetry/state-log.cvs'
    else: 
        logfile: None  

    emergency_server = EmergencyAction(common.config.defaults.drone_emergency_action_name, drone)
    rospy.loginfo("emergency server created")

    launch_server = LaunchAction(common.config.defaults.drone_launch_action_name, drone)
    rospy.loginfo("launch server created")

    move_server = MoveAction(common.config.defaults.drone_move_action_name, drone)
    rospy.loginfo("move server created")

    command_server = CommandAction(common.config.defaults.drone_command_action_name, drone)
    rospy.loginfo("command server created")


    if imagesdir:        
        img = ImageSensor(imagesdir, topic=common.config.defaults.drone_image_sensor_publish_topic_name)
        rospy.Timer(rospy.Duration(common.config.defaults.drone_image_sensor_publish_delay), img.publish)
        rospy.loginfo("image publisher started with images from " + imagesdir)


    if logfile:
        twist = TwistSensor(logfile, topic=common.config.defaults.drone_twist_sensor_publish_topic_name)
    
        rospy.Timer(rospy.Duration(0.1), twist.publish)
        rospy.loginfo("twist publisher started from " + logfile)

        battery = BatterySensor(topic=common.config.defaults.drone_battery_sensor_publish_topic_name)
        rospy.Timer(rospy.Duration(0.1), battery.publish)
        rospy.loginfo("battery publisher started from " + logfile)


    rospy.loginfo("drone_node started")
    rospy.spin()

if __name__ == '__main__':
    try:
        drone_node()
    except rospy.ROSInterruptException:
        drone.terminate()
        rospy.loginfo("drone node terminated")
        pass
