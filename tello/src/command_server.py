#! /usr/bin/env python3

import rospy

import actionlib

import tello.msg

class TelloAction(object):

    # create messages that are used to publish feedback/result
    _launch_feedback = tello.msg.LaunchFeedback()
    _launch_result = tello.msg.LaunchResult()

    _command_feedback = tello.msg.CommandFeedback()
    _command_result = tello.msg.CommandResult()

    _move_feedback = tello.msg.MoveFeedback()
    _move_result = tello.msg.MoveResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, tello.msg.LaunchAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def send_command(self, command):
         rospy.loginfo(command)      

    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(1)
        success = True
        
        # append the seeds for the fibonacci sequence
        self._launch_feedback.progress = []
        self._launch_feedback.progress.append(True)
        
        # publish info to the console for the user
        rospy.loginfo('%s: launching, ence of order %i with sucess %i' % (self._action_name, goal.order, self._launch_feedback.progress[0]))
        
        # start executing the action
        if (goal.order):
            self.send_command("takeoff")
        else:
            self.send_command("land")
        for i in range(1, 3):
            if self._as.is_preempt_requested():
              rospy.loginfo('%s: Preempted' % self._action_name)
              self._as.set_preempted()
              success = False
              break
            self._launch_feedback.progress.append(True)
            # publish the feedback
            self._as.publish_feedback(self._launch_feedback)
            rospy.loginfo("s")
            r.sleep()
          
        self._launch_result.success = success
        rospy.loginfo('%s: End %r' % (self._action_name, self._launch_result))
        self._as.set_succeeded(self._launch_result)
        
if __name__ == '__main__':
    rospy.init_node('launch')
    print("launching server:", rospy.get_name())
    server = LaunchAction(rospy.get_name())
    rospy.spin()
