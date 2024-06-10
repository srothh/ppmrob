#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header

class TwistSensor():

    _drone = None

    def __init__(self, topic='/drone/twist', drone=None):
        self._publish_to_topic = topic
        self._publisher = rospy.Publisher(self._publish_to_topic, TwistStamped, queue_size=10)
        self._drone = drone        
        self._counter = 0
        rospy.loginfo('TwistSensor initalized')

    def publish(self, event=None, state=None):
        try:
            #rospy.loginfo("Publishing TwistStamped from %s" % state)
            if state:
                msg = TwistStamped()
                msg.header = Header()
                msg.header.frame_id = "velocity"
                msg.header.stamp = rospy.Time.now()
                msg.twist.linear.x = state.get('vgx')
                msg.twist.linear.y = state.get('vgy')
                msg.twist.linear.z = state.get('vgz')
                msg.twist.angular.x = state.get('pitch')
                msg.twist.angular.y = state.get('roll')
                msg.twist.angular.z = state.get('yaw')
                self._publisher.publish(msg)
                self._counter += 1
        except Exception as e:
            rospy.loginfo(e)
    

