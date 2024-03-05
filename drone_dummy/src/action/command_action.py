#! /usr/bin/env python3

import rospy

import actionlib
import time

import drone.msg



class CommandAction(object):
   
# create messages that are used to publish feedback/result
    _feedback = drone.msg.CommandFeedback()
    _result = drone.msg.CommandResult()

    def __init__(self, name, tello):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, drone.msg.CommandAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        self._tello = tello



    def execute_cb(self, goal):
        # publish info to the console for the user
        rospy.loginfo('%s: %s' % (self._action_name, goal.command))

        # start executing
        time.sleep(1)

        success = True
        self._result.success = success
        self._as.set_succeeded(self._result)