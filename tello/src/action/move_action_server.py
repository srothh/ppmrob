#! /usr/bin/env python3

import rospy
import actionlib
import time

import tello.msg
from action import ActionServer

from djitellopy import Tello

class MoveActionServer(ActionServer):
   
    _currentaxis = ''

    def __init__(self, name, drone):
        #start action server
        self._feedback = tello.msg.MoveFeedback()
        self._result = tello.msg.MoveResult()
        super().__init__(name,drone, tello.msg.MoveAction)
 


    def execute_cb(self, goal):
        rospy.loginfo('%s %s %d' % (self._action_name, goal.axis, goal.parameter))
        try:
            #axis, distance = goal.command.spit(';', 1)
            axis = goal.axis.strip()
            distance = goal.parameter
            self._currentaxis = axis

            if not isinstance(distance, int):
                raise Exception('invalid distance parameter: %s' % distance)
            
            if axis == 'r':
                if (0 <= distance <= 360):
                    self.command('cw %d' % distance)
                elif (-360 <= distance < 0):
                    self.command('ccw %d' % abs(distance))
                else:
                    raise Exception("invalid rotation bearing: %d" % distance)
            elif axis == 'x':
                if (20 <= distance <= 500):
                    self.command('forward %d' % distance)
                elif (-500 <= distance <= -20):
                    self.command('back %d' % abs(distance))
                else:
                    raise Exception("invalid movement distance: %d" % distance)                                        
            elif axis == 'y':
                if (20 <= distance <= 500):
                    self.command('right %d' % distance)
                elif (-500 <= distance <= -20):
                    self.command('left %d' % abs(distance))
                else:
                    raise Exception("invalid movement distance: %d" % distance)                                        
            elif axis == 'z':
                if (20 <= distance <= 500):
                    self.command('up %d' % distance)
                elif (-500 <= distance <= -20):
                    self.command('down %d' % abs(distance))
                else:
                    raise Exception("invalid movement distance: %d" % distance)                                        
            else:
                raise Exception("invalid movement axis: %s" % axis)
        
        except Exception as e:
            rospy.loginfo(e)
            self.success_cb(False)

    def success_cb(self, success):
        self._result.success = success
        self._as.set_succeeded(self._result)
        rospy.loginfo('end: %s %s' % (self._action_name, success))

    def feedback_cb(self):
        #rospy.loginfo("currentaxis %s" % self._currentaxis)
        value = 0
        if self._currentaxis == 'r':
            value = self._drone.get_state_field('yaw')
        elif self._currentaxis == 'x':
            value = self._drone.get_state_field('pitch')
        elif self._currentaxis == 'y':
            value = self._drone.get_state_field('roll')
        elif self._currentaxis == 'z':
            value = self._drone.get_state_field('vgz')

        #rospy.loginfo("fb %d" % value)
        self._feedback.progress.append(value)

        self._as.publish_feedback(self._feedback)    


if __name__ == '__main__':
    rospy.init_node('y')
    drone = Tello()
    drone.connect()
    print("launching server:", rospy.get_name())
    server = YActionServer(rospy.get_name(), drone)
    rospy.spin()
