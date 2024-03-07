 #! /usr/bin/env python

import rospy


import actionlib 
import drone


class EmergencyAction(object):

    # create messages that are used to publish feedback/result
    _feedback = drone.msg.EmergencyFeedback()
    _result = drone.msg.EmergencyResult()

    def __init__(self, name, tello):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, drone.msg.EmergencyAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        self._tello = tello

    def execute_cb(self, goal):
        # publish info to the console for the user
        rospy.loginfo('%s' % (self._action_name))

        # start executing

        success = False
        if goal.soft:
            # soft landing
            success = self._tello.execute_commands(['stop', 'land'])
        else:
            # hard landing
            success = self._tello.send_command('emergency')

        self._result.success = success
        self._as.set_succeeded(self._result)