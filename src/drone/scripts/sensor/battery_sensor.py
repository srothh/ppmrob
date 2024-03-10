#!/usr/bin/env python3

import rospy
from std_msgs.msg import UInt8

class BatterySensor():

    _drone = None

    def __init__(self, drone=None, topic=None):
        self._publisher = rospy.Publisher(topic, UInt8, queue_size=10)
        self._drone = drone        
        self._counter = 0

    def publish(self, event=None, state=None):
        try:
            if state:
                msg = UInt8()
                msg.data = state.get('bat')
                self._publisher.publish(msg)
                self._counter += 1
        except Exception as e:
            rospy.loginfo(e)
    

