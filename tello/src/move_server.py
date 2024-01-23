#! /usr/bin/env python3

import rospy

import actionlib

import tello.msg

class MoveAction(object):

    # create messages that are used to publish feedback/result
    _feedback = tello.msg.MoveFeedback()
    _result = tello.msg.MoveResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, tello.msg.MoveAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def send_command(self, command):
         rospy.loginfo(command)      

    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(100)
        success = False
        
        # append first feedback
        self._feedback.progress = []
        self._feedback.progress.append(True)
        
        # publish info to the console for the user
        rospy.loginfo('%s: launching, move cm=%i with sucess %i' % (self._action_name, goal.cm, self._feedback.progress[0]))
        
        # start executing the action
        self.send_command("move %u" % goal.cm)
 
        # simulate flying 1 m/sec
        # TODO: how to determine progress and outcome of move
        for i in range(1, goal.cm):
            if self._as.is_preempt_requested():
              rospy.loginfo('%s: Preempted' % self._action_name)
              self._as.set_preempted()
              success = False
              break
            self._launch_feedback.progress.append(True)
            # publish the feedback
            self._as.publish_feedback(self._launch_feedback)
            r.sleep()
        success = True  
        self._launch_result.success = success
        rospy.loginfo('%s: End %r' % (self._action_name, self._launch_result))
        self._as.set_succeeded(self._launch_result)
        
if __name__ == '__main__':
    rospy.init_node('move')
    print("launching server:", rospy.get_name())
    server = LaunchAction(rospy.get_name())
    rospy.spin()
