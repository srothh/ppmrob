#! /usr/bin/env python3

from __future__ import print_function

import rospy
# Brings in the SimpleActionClient
import actionlib
import time

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import tello.msg

# Called every time feedback is received for the goal
def feedbackCb(feedback):  
    #print(".", feedback)
    pass

def launch_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('launch', tello.msg.LaunchAction)

    # Waits until the action server has started up and started
    # listening for goals.
    print('waiting for server')
    client.wait_for_server()
    print('takeoff')
    takeoffgoal = tello.msg.LaunchGoal(order=True)
    client.send_goal(takeoffgoal, feedback_cb=feedbackCb)
    client.wait_for_result()
    # Prints out the result of executing the action
    result = client.get_result()  

    print('rotate')
    rotclient = actionlib.SimpleActionClient('rotate', tello.msg.RotateAction)
    rotclient.wait_for_server()
    rotclient.send_goal_and_wait(tello.msg.RotateGoal(bearing=90))
    rotclient.send_goal_and_wait(tello.msg.RotateGoal(bearing=90))
    rotclient.send_goal_and_wait(tello.msg.RotateGoal(bearing=-180))

    print('x')
    xclient = actionlib.SimpleActionClient('x', tello.msg.XAction)
    xclient.wait_for_server()
    xclient.send_goal_and_wait(tello.msg.XGoal(distance=50))
    #time.sleep(1)
    xclient.send_goal_and_wait(tello.msg.XGoal(distance=-50))


    print('y')
    yclient = actionlib.SimpleActionClient('y', tello.msg.YAction)
    yclient.wait_for_server()
    yclient.send_goal_and_wait(tello.msg.YGoal(distance=50))
    yclient.send_goal_and_wait(tello.msg.YGoal(distance=-50))

    print('z')
    zclient = actionlib.SimpleActionClient('z', tello.msg.ZAction)
    zclient.wait_for_server()
    zclient.send_goal_and_wait(tello.msg.ZGoal(distance=50))
    zclient.send_goal_and_wait(tello.msg.ZGoal(distance=-50))

    print('command')
    zclient = actionlib.SimpleActionClient('command', tello.msg.CommandAction)
    zclient.wait_for_server()
    zclient.send_goal_and_wait(tello.msg.CommandGoal(command='flip b'))

    #test keepalive
    time.sleep(15)

    print('land')
    landgoal = tello.msg.LaunchGoal(order=False)
    client.send_goal(landgoal, feedback_cb=feedbackCb)
    client.wait_for_result()
    result = client.get_result()
    print(result)
    

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('launch_client_py')
        result = launch_client()
        print("Result:", ', ', result)
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
