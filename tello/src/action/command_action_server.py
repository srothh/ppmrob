#! /usr/bin/env python3

import rospy

import actionlib
import time

import tello.msg
from djitellopy import Tello
from action import ActionServer

class CommandActionServer(ActionServer):
   

    def __init__(self, name, drone):
        #start action server
        self._feedback = tello.msg.CommandFeedback()
        self._result = tello.msg.CommandResult()
        super().__init__(name, drone, tello.msg.CommandAction)
 


    def execute_cb(self, goal):
        rospy.loginfo('%s %s' % (self._action_name, goal.command))
        self._feedback.progress = []
        self._feedback.progress.append(True)        
        
        success = self._drone.send_control_command(goal.command)

        self.success_cb(success)


    def success_cb(self, success):
        self._result.success = success
        rospy.loginfo('%s: End %r' % (self._action_name, self._result))
        self._as.set_succeeded(self._result)


    def feedback_cb(self):
        print('.', end =" ")
        self._feedback.progress.append(True)
        self._as.publish_feedback(self._feedback)    

        
if __name__ == '__main__':
    rospy.init_node('command')
    print("launching server:", rospy.get_name())
    server = CommandActionServer(rospy.get_name())
    rospy.spin()
