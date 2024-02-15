#!/usr/bin/env python3

import rospy
from geometry_msgs import TwistStamped
from std_msgs.msg import Header
def TelemetrySensor():

    _drone = None

    def __init__(self, drone):
        self._publisher = rospy.Publisher('/drone/camera', Image, queue_size=10)
        self._drone = drone        
        #self.drone.startStateHandler(self.process)

    def publish(self, event=None):
        try:
            state = _drone.getLastState()
            twist = get_twist(state)
        except Exception as e:
            rospy.loginfo(e)
    
    def get_twist(self, state):
        msg = TwistStamped()
        msg.header = Header()
        msg.header.frame_id = "velocity"
        msg.header.stamp = rospy.Time.now()
        msg.twist.linear.x = state.vgx
        msg.twist.linear.y = state.vgy
        msg.twist.linear.z = state.vgz


