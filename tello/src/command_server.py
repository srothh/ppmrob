#! /usr/bin/env python3

import rospy

import actionlib

import tello.msg

class CommandAction(object):

    # create messages that are used to publish feedback/result
    _feedback = tello.msg.CommandFeedback()
    _result = tello.msg.CommandResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, tello.msg.CommandAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def send_command(self, command):
         #TODO: send command
         rospy.loginfo(command)      

    def execute_cb(self, goal):
        # helper variables
        success = True
        
        # append the seeds for the fibonacci sequence
        self._feedback.progress = []
        self._feedback.progress.append(True)
        
        # publish info to the console for the user
        rospy.loginfo('%s: command %s with sucess %i' % (self._action_name, goal.command, self._launch_feedback.progress[0]))
        
        # start executing the action
        self.send_command(goal.command)

        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted()
            success = False
        
        self._feedback.progress.append(success)
        # publish the feedback
        self._as.publish_feedback(self._feedback)
          
        self._result.success = success
        rospy.loginfo('%s: End %r' % (self._action_name, self._result))
        self._as.set_succeeded(self._result)
        
if __name__ == '__main__':
    rospy.init_node('launch')
    print("launching server:", rospy.get_name())
    server = CommandAction(rospy.get_name())
    rospy.spin()
