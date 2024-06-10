#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import BatteryState


class BatterySensor:

    _drone = None

    def __init__(self, drone=None, topic=None):
        self._publisher = rospy.Publisher(topic, BatteryState, queue_size=10)
        self._drone = drone
        self._counter = 0

    def publish(self, event=None, state=None):
        try:
            if state:
                msg = BatteryState()
                msg.percentage = state.get("bat")
                self._publisher.publish(msg)
                self._counter += 1
        except Exception as e:
            rospy.loginfo(e)
