#!/usr/bin/env python3

import rospy
from std_msgs.msg import UInt8
import time

class BatterySensor():

    _drone = None

    def __init__(self, drone=None, topic=None):
        self._publisher = rospy.Publisher(topic, UInt8, queue_size=10)
        self._counter = 0
        self._start_time = time.time()

    def publish(self, event=None, state=None):
        try:
            msg = UInt8()
            val = round(99 - ((time.time() - self._start_time) / 6))
            msg.data = val if val > 0 else 0
            self._publisher.publish(msg)
            self._counter += 1
        except Exception as e:
            rospy.loginfo(e)
    

