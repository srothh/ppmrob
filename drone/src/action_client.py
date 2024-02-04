#!/usr/bin/env python3

import rospy
import actionlib
import time
import drone.msg

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


    print('land')  
    client.send_goal_and_wait(drone.msg.LaunchGoal(takeoff=False))
    print(client.get_result())
    
if __name__ == '__main__':
    try:
        rospy.init_node('launch_client_py')
        launch_client()
    except rospy.ROSInterruptException:
        pass

