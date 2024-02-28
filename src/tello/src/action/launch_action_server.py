#! /usr/bin/env python3

import rospy
import actionlib
import time

import tello.msg
from action import ActionServer


from djitellopy.tello import Tello

class LaunchActionServer(ActionServer):
    # create messages that are used to publish feedback/result
    _feedback = tello.msg.LaunchFeedback()
    _result = tello.msg.LaunchResult()
    _drone = None


    def __init__(self, name, drone):
        #start action server
        self._feedback = tello.msg.LaunchFeedback()
        self._result = tello.msg.LaunchResult()        
        super().__init__(name,drone, tello.msg.LaunchAction)

 
    def execute_cb(self, goal):
        rospy.loginfo('%s %i' % (self._action_name, goal.order))

        # append the seeds for the fibonacci sequence
        self._feedback.progress = []
        self._feedback.progress.append(True)
        
        if (goal.order):
            success = self.command('takeoff')
        else:
            success = self.command('land')

    def success_cb(self, success):
        self._result.success = success
        self._as.set_succeeded(self._result)
        rospy.loginfo('end: %s %s' % (self._action_name, success))

    def feedback_cb(self):
        print('.', end =" ")
        self._feedback.progress.append(True)
        self._as.publish_feedback(self._feedback) 
          
      
if __name__ == '__main__':
    rospy.init_node('launch')
    drone = Tello()
    drone.connect()
    rospy.loginfo("launching server: %s", rospy.get_name())
    server = LaunchActionServer(rospy.get_name(), drone)
    rospy.spin()
