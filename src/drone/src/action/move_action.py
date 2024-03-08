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
        success = False
        try:
            x = round(goal.target.translation.x)
            y = round(goal.target.translation.y)
            z = round(goal.target.translation.z)
            rotation = round(goal.target.rotation.z)

            if rotation != 0:
                if (0 <= rotation <= 360):
                    success = self.command('cw %d' % rotation)
                elif (-360 <= rotation < 0):
                    success = self.command('ccw %d' % abs(rotation))
                else:
                    raise Exception("invalid rotation bearing: %d" % rotation)
            elif x != 0:
                distance = x
                if (20 <= distance <= 500):
                    success = self.command('forward %d' % distance)
                elif (-500 <= distance <= -20):
                    success = self.command('back %d' % abs(distance))
                else:
                    raise Exception("invalid movement distance: %d" % distance)                                        
            elif y != 0:
                distance = y
                if (20 <= distance <= 500):
                    success = self.command('right %d' % distance)
                elif (-500 <= distance <= -20):
                    success = self.command('left %d' % abs(distance))
                else:
                    raise Exception("invalid movement distance: %d" % distance)                                        
            elif z != 0:
                distance = z
                if (20 <= distance <= 500):
                    success = self.command('up %d' % distance)
                elif (-500 <= distance <= -20):
                    success = self.command('down %d' % abs(distance))
                else:
                    raise Exception("invalid movement distance: %d" % distance)                                        
            else:
                raise Exception("invalid movement axis: %s" % goal.target.translation)
        
            self.success_cb(success)
    
        except Exception as e:
            rospy.loginfo(e)
            self.success_cb(False)

    def command(self, command):
        return self._drone.command(command)

    def success_cb(self, success):
        self._result.success = success
        self._as.set_succeeded(self._result)
        rospy.loginfo('end: %s %s' % (self._action_name, success))

