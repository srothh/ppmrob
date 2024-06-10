#!/usr/bin/env python3

import rospy, time
from sensor_msgs.msg import BatteryState

class BatterySensor:

    _drone = None

    def __init__(self, drone=None, topic=None):
        self._publisher = rospy.Publisher(topic, BatteryState, queue_size=10)
        self._start_time = time.time()
        self._counter = 0

    def publish(self, event=None, state=None):
        try:
            msg = BatteryState()
            val = round(99 - ((time.time() - self._start_time) / 6))
            msg.percentage = val if val > 0 else 0
            self._publisher.publish(msg)
            self._counter += 1
        except Exception as e:
            rospy.loginfo(e)
    

