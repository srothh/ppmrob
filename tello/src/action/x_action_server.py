#! /usr/bin/env python3

import rospy
import actionlib
import time

import tello.msg
from action import ActionServer

from djitellopy import Tello

class XActionServer(ActionServer):
   

    def __init__(self, name, drone):
        #start action server
        self._feedback = tello.msg.XFeedback()
        self._result = tello.msg.XResult()
        super().__init__(name,drone, tello.msg.XAction)
 


    def execute_cb(self, goal):
        rospy.loginfo('%s %i' % (self._action_name, goal.distance))
        
        if (20 <= goal.distance <= 500):
            self.command('forward %d' % goal.distance)
        elif (-20 >= goal.distance >= -500):
            self.command('back %d' % abs(goal.distance))
        else:
            rospy.loginfo("distance out of range: %d", goal.distance)
            self.success_cb(False)


    def success_cb(self, success):
        self._result.success = success
        self._as.set_succeeded(self._result)
        rospy.loginfo('end: %s %s' % (self._action_name, success))

    def feedback_cb(self):
        print('.', end =" ")
        self._feedback.progress.append(self._drone.get_state_field('vgx'))
        self._as.publish_feedback(self._feedback)    


if __name__ == '__main__':
    rospy.init_node('x')
    drone = Tello()
    drone.connect()
    print("launching server:", rospy.get_name())
    server = XActionServer(rospy.get_name(), drone)
    rospy.spin()
