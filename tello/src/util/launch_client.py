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
    # print last element of feedback
    print(feedback.progress[-1], end =".")

    

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

    # print('rotate')
    # rotclient = actionlib.SimpleActionClient('rotate', tello.msg.RotateAction)
    # rotclient.wait_for_server()
    # rotclient.send_goal_and_wait(tello.msg.RotateGoal(bearing=90))
    # rotclient.send_goal_and_wait(tello.msg.RotateGoal(bearing=90))
    # rotclient.send_goal_and_wait(tello.msg.RotateGoal(bearing=-180))

    # print('x')
    # xclient = actionlib.SimpleActionClient('x', tello.msg.XAction)
    # xclient.wait_for_server()
    # xclient.send_goal(tello.msg.XGoal(distance=50), feedback_cb=feedbackCb)
    # xclient.wait_for_result()
    # #time.sleep(1)
    # xclient.send_goal_and_wait(tello.msg.XGoal(distance=-50))


    # print('y')
    # yclient = actionlib.SimpleActionClient('y', tello.msg.YAction)
    # yclient.wait_for_server()
    # yclient.send_goal_and_wait(tello.msg.YGoal(distance=50))
    # yclient.send_goal_and_wait(tello.msg.YGoal(distance=-50))

    # print('z')
    # zclient = actionlib.SimpleActionClient('z', tello.msg.ZAction)
    # zclient.wait_for_server()
    # zclient.send_goal_and_wait(tello.msg.ZGoal(distance=50))
    # zclient.send_goal_and_wait(tello.msg.ZGoal(distance=-50))


    print('\nmove')
    moveclient = actionlib.SimpleActionClient('move', tello.msg.MoveAction)
    moveclient.wait_for_server()
    moveclient.send_goal(tello.msg.MoveGoal(axis='x', parameter=50), feedback_cb=feedbackCb)
    moveclient.wait_for_result()
    moveclient.send_goal(tello.msg.MoveGoal(axis='x', parameter=-50), feedback_cb=feedbackCb)
    moveclient.wait_for_result()
    moveclient.send_goal(tello.msg.MoveGoal(axis='y', parameter=50), feedback_cb=feedbackCb)
    moveclient.wait_for_result()
    moveclient.send_goal(tello.msg.MoveGoal(axis='y', parameter=-50), feedback_cb=feedbackCb)
    moveclient.wait_for_result()
    moveclient.send_goal(tello.msg.MoveGoal(axis='z', parameter=-50), feedback_cb=feedbackCb)
    moveclient.wait_for_result()
    moveclient.send_goal(tello.msg.MoveGoal(axis='z', parameter=50), feedback_cb=feedbackCb)
    moveclient.wait_for_result()
    # #time.sleep(1)
    # xclient.send_goal_and_wait(tello.msg.XGoal(distance=-50))


    #print('command')
    #zclient = actionlib.SimpleActionClient('command', tello.msg.CommandAction)
    #zclient.wait_for_server()
    #zclient.send_goal_and_wait(tello.msg.CommandGoal(command='flip b'))

    #test keepalive
    #time.sleep(15)

    print('\nland')
    landgoal = tello.msg.LaunchGoal(order=False)
    client.send_goal(landgoal, feedback_cb=feedbackCb)
    client.wait_for_result()
    result = client.get_result()
    print(result)
    

def launch_client_2():
    client = actionlib.SimpleActionClient('launch', tello.msg.LaunchAction)

    print('waiting for server')
    client.wait_for_server()
    print('takeoff')
    takeoffgoal = tello.msg.LaunchGoal(order=True)
    client.send_goal(takeoffgoal, feedback_cb=feedbackCb)
    client.wait_for_result()
    result = client.get_result()  


    print('command')
    cclient = actionlib.SimpleActionClient('command', tello.msg.CommandAction)
    cclient.wait_for_server()
    cclient.send_goal_and_wait(tello.msg.CommandGoal(command='speed 10'))


    print('\nmove')
    moveclient = actionlib.SimpleActionClient('move', tello.msg.MoveAction)
    moveclient.wait_for_server()
    moveclient.send_goal(tello.msg.MoveGoal(axis='x', parameter=100), feedback_cb=feedbackCb)
    #moveclient.wait_for_result()
    print('sleep')
    time.sleep(5)
    moveclient.send_goal(tello.msg.MoveGoal(axis='x', parameter=-100), feedback_cb=feedbackCb)



    print('\nland')
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
        #result = launch_client()
        #print("Result:", ', ', result)
        result = launch_client_2()
        print("Result:", ', ', result)
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
