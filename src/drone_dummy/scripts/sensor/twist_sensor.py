#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header
import common.config.defaults
import time
import pandas as pd

class TwistSensor():

    _drone = None

    def __init__(self, datafile):
        self._publish_to_topic = common.config.defaults.drone_twist_sensor_publish_topic_name
        self._publisher = rospy.Publisher(self._publish_to_topic, TwistStamped, queue_size=10)
        self._datafile = datafile     
        self._counter = 0
        self.idx = 0
        # read data

        self.df = pd.read_csv(self._datafile, delimiter=';', decimal=".")
        #df = pd.read_csv('data/tv_telemetry_210327_131735_demo_orbit_party.txt',decimal=".")
        self.idx = 0 

        rospy.loginfo('TwistSensor initalized')

    def publish(self, event=None, state=None):
        try:

            msg = TwistStamped()
            msg.header = Header()
            msg.header.frame_id = "velocity"
            msg.header.stamp = rospy.Time.now()
            msg.twist.linear.x = self.df['vgx'][self.idx]
            msg.twist.linear.y = self.df['vgy'][self.idx]
            msg.twist.linear.z = self.df['vgz'][self.idx]
            msg.twist.angular.x = self.df['roll'][self.idx]
            msg.twist.angular.y = self.df['pitch'][self.idx]
            msg.twist.angular.z = self.df['yaw'][self.idx]
            #msg.twist.angular.z = -90

            self._publisher.publish(msg)
            self._counter += 1
            self.idx = (self.idx + 1) % len(self.df)
        except Exception as e:
            rospy.loginfo(e)
    

