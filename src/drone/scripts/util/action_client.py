#!/usr/bin/env python3
import sys
print('Updated sys.path:', sys.path)

import rospy
import actionlib
import time
from common import msg
from geometry_msgs.msg import Transform, Vector3, Quaternion

def launch_client():
    print('start')
    client = actionlib.SimpleActionClient('launch', drone.msg.LaunchAction)

    print('waiting for server')
    client.wait_for_server()
    print('takeoff')
    takeoffgoal = drone.msg.LaunchGoal(takeoff=True)
    client.send_goal(takeoffgoal)
    client.wait_for_result()
    # Prints out the result of executing the action
    print(client.get_result())


    moveclient = actionlib.SimpleActionClient('move', drone.msg.MoveAction)
    moveclient.wait_for_server()

    moveclient.send_goal_and_wait(drone.msg.MoveGoal(target=Transform(Vector3(0, 0, 0), Quaternion(0, 0, 90, 0))))
    time.sleep(10)
    moveclient.send_goal_and_wait(drone.msg.MoveGoal(target=Transform(Vector3(0, 0, 0), Quaternion(0, 0, -90, 0))))
    time.sleep(10)
    moveclient.send_goal_and_wait(drone.msg.MoveGoal(target=Transform(Vector3(50, 0, 0), Quaternion(0, 0, 0, 0))))
    time.sleep(10)
    moveclient.send_goal_and_wait(drone.msg.MoveGoal(target=Transform(Vector3(-50, 0, 0), Quaternion(0, 0, 0, 0))))
    time.sleep(10)
    moveclient.send_goal_and_wait(drone.msg.MoveGoal(target=Transform(Vector3(0, 50, 0), Quaternion(0, 0, 0, 0))))
    time.sleep(10)
    moveclient.send_goal_and_wait(drone.msg.MoveGoal(target=Transform(Vector3(0, -50, 0), Quaternion(0, 0, 0, 0))))
    time.sleep(10)
    moveclient.send_goal_and_wait(drone.msg.MoveGoal(target=Transform(Vector3(0, 0, -50), Quaternion(0, 0, 0, 0))))
    time.sleep(10)
#    moveclient.send_goal_and_wait(drone.msg.MoveGoal(target=Transform(Vector3(0, 0, 50), Quaternion(0, 0, 0, 0))))
 #   time.sleep(10)

    #commandclient = actionlib.SimpleActionClient('command', drone.msg.CommandAction)
    #commandclient.wait_for_server()
    #commandclient.send_goal_and_wait(drone.msg.CommandGoal(command='flip b'))
    #time.sleep(5)
    #commandclient.send_goal_and_wait(drone.msg.CommandGoal(command='flip f'))
    #time.sleep(5)
    #commandclient.send_goal_and_wait(drone.msg.CommandGoal(command='flip r'))
    #time.sleep(5)
    #commandclient.send_goal_and_wait(drone.msg.CommandGoal(command='flip l'))
    #time.sleep(5)


    print('land')  
    client.send_goal_and_wait(drone.msg.LaunchGoal(takeoff=False))
    print(client.get_result())
    
if __name__ == '__main__':
    try:
        rospy.init_node('launch_client_py')
        launch_client()
    except rospy.ROSInterruptException:
        pass

