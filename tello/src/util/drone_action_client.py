#! /usr/bin/env python3

from __future__ import print_function

import rospy
# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import tello.msg

class DroneActionClient:
    def __init__(self):
        # Creates the SimpleActionClient, passing the type of the action
        # (FibonacciAction) to the constructor.
        self.launch_client = actionlib.SimpleActionClient('launch', tello.msg.LaunchAction)
        self.move_client = actionlib.SimpleActionClient('move', tello.msg.MoveAction)
        print("waiting for server")
        self.move_client.wait_for_server()
        self.launch_client.wait_for_server()

        self.queue = []
        self.retry = 3

    #movements
    def takeoff(self):
        self.launch(True)
    def land(self):
        self.launch(False)
    def move_x(self, distance):
        #self.queue.append(tello.msg.MoveGoal(axis='x', parameter=distance))
        self.execute_goal_and_wait(self.move_client, tello.msg.MoveGoal(axis='x', parameter=distance))
    def move_y(self, distance):
        #self.queue.append(tello.msg.MoveGoal(axis='y', parameter=distance))
        self.execute_goal_and_wait(self.move_client, tello.msg.MoveGoal(axis='y', parameter=distance))
    def move_z(self, distance):
        #self.queue.append(tello.msg.MoveGoal(axis='z', parameter=distance))
        self.execute_goal_and_wait(self.move_client, tello.msg.MoveGoal(axis='z', parameter=distance))
    def rotate(self, bearing):
        #self.queue.append(tello.msg.MoveGoal(axis='r', parameter=bearing))
        self.execute_goal_and_wait(self.move_client, tello.msg.MoveGoal(axis='r', parameter=bearing))


    # execute first goal in the queue
    def execute(self):
        if (self.queue != []):            
            goal = self.queue[0]    
            rospy.loginfo('executing goal %s' % goal)      
            self.move_client.send_goal(goal, self.doneCb, self.activeCb, self.feedbackCb)
        else:
            rospy.loginfo('queue empty')


    def launch(self, order=True):
        goal = tello.msg.LaunchGoal(order=order)
        self.execute_goal_and_wait(self.launch_client, goal)

    def move(self, axis, distance):
        goal = tello.msg.MoveGoal(axis=axis, parameter=distance)
        self.execute(self.move_client, goal)

    def execute_goal_and_wait(self, client, goal, retries=3):
        rospy.loginfo('executing goal: %s' % goal)
        for i in range(0, retries):
            client.send_goal_and_wait(goal)
            result = client.get_result()   
            success = result.success
            if success:
                return success  
            else:
                rospy.loginfo('unsuccessfull, retrying %d' % i)   
        

    #callbacks

    def doneCb(self, state, result):
        print("done", state, result)
        # if result.success:
        #     # execute next goal in the queue
        #     self.queue = self.queue[1:]
        #     self.retry = 3
        # else:            
        #     # retry
        #     self.retry = self.retry - 1
        #     rospy.loginfo('retrying %d' % self.retry)
        # self.execute()


    def activeCb(self):
        print("active")

    def feedbackCb(self, feedback):  
        print(feedback.progress[-1], end=".")


