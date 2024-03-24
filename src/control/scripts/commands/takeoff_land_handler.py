#!/usr/bin/env python3

import rospy
import actionlib
from drone.msg import CommandAction, CommandGoal


class TakeOffAndLandHandler:
    def __init__(self):

        # self.take_off_action_client = actionlib.SimpleActionClient('TakeOff', TakeOffAction)
        # self.take_off_action_client.wait_for_server()
        # self.land_client = actionlib.SimpleActionClient('Land', LandAction)
        self.command_client = actionlib.SimpleActionClient('command', CommandAction)
        self.command_client.wait_for_server()

    def handle_takeoff(self):
        """ Sends takeoff action message to the drone action server
        @param takeoff:
        """
        goal = CommandGoal(command="takeoff")
        self.command_client.send_goal(goal)
        self.command_client.wait_for_result()
        result = self.command_client.get_result()

        if result.success:
            rospy.loginfo("Drone successfully took off")
            takeoff = True
        else:
            rospy.loginfo("Drone takeoff failed")

    def handle_land(self):
        """ Sends landing action message to the drone action server

        @param land:
        """
        goal = CommandGoal(command="land")
        self.command_client.send_goal(goal)
        self.command_client.wait_for_result()
        result = self.command_client.get_result()

        if result.success:
            rospy.loginfo("Drone successfully landed")
            land = True
        else:
            rospy.loginfo("Drone landing failed")
