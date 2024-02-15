 #! /usr/bin/env python

import rospy


import actionlib 
import drone.msg


class LaunchAction(object):

    # create messages that are used to publish feedback/result
    _feedback = drone.msg.LaunchFeedback()
    _result = drone.msg.LaunchResult()

    def __init__(self, name, tello):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, drone.msg.LaunchAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        self._tello = tello

    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(1)
        success = True

        # publish info to the console for the user
        rospy.loginfo('%s: Executing, launching drone' % (self._action_name))

        # start executing

        success = False
        if goal.takeoff:
            # takeoff
            success = self._tello.command('takeoff')
        else:
            # land
            success = self._tello.command('land')

        self._result.success = success
        self._as.set_succeeded(self._result)