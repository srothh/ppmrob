#! /usr/bin/env python3

from __future__ import print_function

import rospy
# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import tello.msg

# Called once when the goal completes
def doneCb(state, result):
    print("done", state, result)
 
# Called once when the goal becomes active    
def activeCb():
    print("active")

# Called every time feedback is received for the goal
def feedbackCb(feedback):  
    print("feedback", feedback)
    
def launch_client(order=True):
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('launch', tello.msg.LaunchAction)

    # Waits until the action server has started up and started
    # listening for goals.
    print("waiting for server")
    client.wait_for_server()
    print("server")

    # Creates a goal to send to the action server.
    goal = tello.msg.LaunchGoal(order=order)

    # Sends the goal to the action server.
    client.send_goal(goal, doneCb, activeCb, feedbackCb)

    # Waits for the server to finish performing the action.
    #client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('launch_client_callbacks_py')
        result = launch_client()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
