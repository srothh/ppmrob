#! /usr/bin/env python3

import rospy
import actionlib
import time
import drone.msg
from tello import Tello
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
        try:
            translation = goal.target.translation
            rotation = goal.target.rotation

            if rotation.z != 0:
                if (0 <= rotation.z <= 360):
                    self.command('cw %d' % rotation.z)
                elif (-360 <= rotation.z < 0):
                    self.command('ccw %d' % abs(rotation.z))
                else:
                    raise Exception("invalid rotation bearing: %d" % rotation.z)
            elif translation.x != 0:
                distance = translation.x
                if (20 <= distance <= 500):
                    self.command('forward %d' % distance)
                elif (-500 <= translation.x <= -20):
                    self.command('back %d' % abs(distance))
                else:
                    raise Exception("invalid movement distance: %d" % distance)                                        
            elif translation.y != 0:
                distance = translation.y
                if (20 <= distance <= 500):
                    self.command('right %d' % distance)
                elif (-500 <= distance <= -20):
                    self.command('left %d' % abs(distance))
                else:
                    raise Exception("invalid movement distance: %d" % distance)                                        
            elif translation.z != 0:
                distance = translation.z
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
        
        self.success_cb(True)

    def command(self, command):
        self._drone.send_command(command)

    def success_cb(self, success):
        self._result.success = success
        self._as.set_succeeded(self._result)
        rospy.loginfo('end: %s %s' % (self._action_name, success))

