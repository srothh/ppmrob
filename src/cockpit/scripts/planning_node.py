#!/usr/bin/env python3

import math
from enum import IntEnum
import numpy as np
import matplotlib.pyplot as plt
import rospy
import actionlib
import time
import drone.msg
import control.msg
from std_msgs.msg import Int32
from geometry_msgs.msg import Transform, Vector3, Quaternion, Pose, Point
from functools import total_ordering
from scipy.spatial.transform import Rotation as Rot
import common.config.defaults as config
from control.msg import (
    MoveResult,
    MoveFeedback,
    PlanningMoveAction,
    PlanningMoveGoal,
    PlanningMoveResult,
    PlanningCommandAction,
    PlanningCommandResult
)

def cockpit_waypoint_callback(msg):
    global waypoint_list
    waypoint_list.append(msg)

def main():  # pragma: no cover
    global waypoint_list
    print("start!!")

    # ox = [0.0, 20.0, 50.0, 100.0, 130.0, 40.0, 0.0]
    # oy = [0.0, -20.0, 0.0, 30.0, 60.0, 80.0, 0.0]
    # resolution = 5.0
    # planning_animation(ox, oy, resolution)

    #rectangle map
    #ox = [0.0, 200.0, 200.0, 0.0, 0.0]
    #oy = [0.0, 0.0, 200.0, 200.0, 0.0]

    #ox = [0.0, 0.0, 300.0, 300.0, 200.0, 200.0, 100.0, 100.0, 0.0]
    #oy = [0.0, 300.0, 300.0, 0.0, 0.0, 100.0, 100.0, 0.0, 0.0]

    #ox = [0.0, 200.0, 200.0, 100.0, 100.0, 0.0,   0]
    #oy = [0.0, 0,     200.0, 200.0, 100.0, 100.0, 0]

    #ox = [0.0, 0.0, 3000.0, 3000.0, 2000.0, 2000.0, 1000.0, 1000.0, 0.0]
    #oy = [0.0, 3000.0, 3000.0, 0.0, 0.0, 1000.0, 1000.0, 0.0, 0.0]


    resolution = 40.0
    #px, py = planning(ox, oy, resolution)

    px = [180,   0 , 180, 0 ]
    py = [0  , 100 , 100, 0 ]

    #px = [20, 0]
    #py = [20  , 0]

#    if do_animation:
#        plt.cla()
#        plt.plot(px, py, "-r")
#        plt.axis("equal")
#        plt.grid(True)
#        plt.savefig("plan.png")
#        plt.close()

    # action_client.py
    # print("start action")
    # client = actionlib.SimpleActionClient("launch", drone.msg.LaunchAction)
    # client.wait_for_server()
    # print("takeoff")
    # takeoffgoal = drone.msg.LaunchGoal(takeoff=True)
    # client.send_goal(takeoffgoal)
    # client.wait_for_result()
    # # Prints out the result of executing the action
    # print(client.get_result())

    #control_transform_client = actionlib.SimpleActionClient("TransformActionServer", drone.msg.ControlTransformAction)
    control_transform_client =  actionlib.SimpleActionClient(config.Control.MOVE_ACTION_NAMESPACE, PlanningMoveAction)

    rospy.loginfo('waiting for control action server..')
    control_transform_client.wait_for_server()
    rospy.loginfo('done')
    drone_command_client = actionlib.SimpleActionClient(config.drone_launch_action_name, drone.msg.LaunchAction)
    rospy.loginfo('waiting for drone command action server..')
    drone_command_client.wait_for_server()
    rospy.loginfo('done')
    
    waypoint_subscriber = rospy.Subscriber('/cockpit/waypoint', Pose, callback=cockpit_waypoint_callback)

    waypoint_list = []

    #px.reverse()
    #py.reverse()
    #wait for all modules to load
    while True:
        time.sleep(5)
        #takeoff
        if len(waypoint_list) > 0:
            takeoff = False
            while not takeoff:
                #drone_command_client.send_goal_and_wait(drone.msg.CommandGoal(command=drone.msg.CommandGoal.TAKEOFF))
                drone_command_client.send_goal_and_wait(drone.msg.LaunchGoal(takeoff=True))
                success = drone_command_client.get_result()
                rospy.loginfo('takeoff: %s' % success)
                takeoff = success
                time.sleep(5)
            while True:
                waypoint = waypoint_list.pop(0)
                #state = control_transform_client.send_goal_and_wait(drone.msg.ControlTransformGoal(target=Transform(Vector3(waypoint.position.x, waypoint.position.y, 0), Quaternion(0, 0, 0, 0))))
                state = control_transform_client.send_goal_and_wait(PlanningMoveGoal(target=[Point(waypoint.position.x, waypoint.position.y, 0)]))
                success = control_transform_client.get_result()
                rospy.loginfo('waypoint: %d %d %s' % (waypoint.position.x, waypoint.position.y, success))
                if waypoint_list == []:
                    drone_command_client.send_goal_and_wait(drone.msg.LaunchGoal(takeoff=False))
                    success = drone_command_client.get_result()
                    rospy.loginfo('land: %s' % success)
                    break                                                                                  

    print("done!!")


if __name__ == "__main__":
    try:
        rospy.init_node("planning_node")
        main()
    except rospy.ROSInterruptException:
        pass
