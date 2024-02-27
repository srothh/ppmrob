#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header

class TwistSensor():

    _drone = None

    def __init__(self, drone=None):
        self._publisher = rospy.Publisher('/drone/twist', TwistStamped, queue_size=2)
        self._drone = drone        
        self._counter = 0
        #self.drone.startStateHandler(self.process)

    def publish(self, event=None, state=None):
        try:
            #state = self._drone.getLastState()
            #rospy.loginfo("Publishing twist from %s" % state)
            if state:
                msg = TwistStamped()
                msg.header = Header()
                msg.header.frame_id = "velocity"
                msg.header.stamp = rospy.Time.now()
                msg.twist.linear.x = state.get('vgx')
                msg.twist.linear.y = state.get('vgy')
                msg.twist.linear.z = state.get('vgz')
                self._publisher.publish(msg)
                self._counter += 1
        except Exception as e:
            rospy.loginfo(e)
    

