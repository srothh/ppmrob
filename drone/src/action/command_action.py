#! /usr/bin/env python3

import rospy

import actionlib
import time
#import common.msg
import common.msg


class CommandAction(object):
   
# create messages that are used to publish feedback/result
    _feedback = common.msg.CommandFeedback()
    _result = common.msg.CommandResult()

    def __init__(self, name, tello):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, common.msg.CommandAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        self._tello = tello



    def execute_cb(self, goal):
        # publish info to the console for the user
        rospy.loginfo('%s: %s' % (self._action_name, goal.command))

        # start executing

        success = False
        if goal.command:
            success = self._tello.command(goal.command)
        else:
            rospy.loginfo('No command given')
        self._result.success = success
        self._as.set_succeeded(self._result)