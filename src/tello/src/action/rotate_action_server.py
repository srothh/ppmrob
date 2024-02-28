#! /usr/bin/env python3

import rospy
import actionlib
import time

import tello.msg
from action import ActionServer


from djitellopy import Tello

class RotateActionServer(ActionServer):
   

    def __init__(self, name, drone):
        #start action server
        self._feedback = tello.msg.RotateFeedback()
        self._result = tello.msg.RotateResult()
        super().__init__(name,drone,tello.msg.RotateAction)


    def execute_cb(self, goal):
        rospy.loginfo('%s %i' % (self._action_name, goal.bearing))
        self._feedback.progress = []
        self._feedback.progress.append(True)
        
        if (0 <= goal.bearing <= 360):
            self.command('cw %d' % goal.bearing) 
        elif (0 > goal.bearing >= -360):
            self.command('ccw %d' % abs(goal.bearing))
        else:
            rospy.loginfo("bearing out of range: %d", goal.bearing)
            self.success_cb(False)

    def success_cb(self, success):
        self._result.success = success
        self._as.set_succeeded(self._result)       
        rospy.loginfo('end: %s %s' % (self._action_name, success))


    def feedback_cb(self):
        print('.', end =" ")
        self._feedback.progress.append(True)
        self._as.publish_feedback(self._feedback)    



if __name__ == '__main__':
    rospy.init_node('rotate')
    drone = Tello()
    drone.connect()
    print("launching server:", rospy.get_name())
    server = RotateActionServer(rospy.get_name(), drone)
    rospy.spin()
