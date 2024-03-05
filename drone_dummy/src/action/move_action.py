#! /usr/bin/env python3

import rospy
import actionlib
import time
 
import drone.msg
#from geometry_msgs.msg import Transform, Translation, Quaternion

class MoveAction(object):
   

    def __init__(self, name, tello):
        #start action server
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, drone.msg.MoveAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        self._drone = tello

        self._feedback = drone.msg.MoveFeedback()
        self._result = drone.msg.MoveResult() 

    def execute_cb(self, goal):
        rospy.loginfo('%s %s' % (self._action_name, goal.target))
        time.sleep(3+(goal.target.translation.x+goal.target.translation.y+goal.target.translation.z)/100)

        self.success_cb(True)

    def command(self, command):
        return self._drone.command(command)

    def success_cb(self, success):
        self._result.success = success
        self._as.set_succeeded(self._result)
        rospy.loginfo('end: %s %s' % (self._action_name, success))

