#!/usr/bin/env python3

import rospy
import actionlib
from control.msg import TakeOffAction, TakeOffGoal, LandAction, LandGoal


class TakeOffAndLandHandler:
    def __init__(self):

        self.take_off_action_client = actionlib.SimpleActionClient('TakeOff', TakeOffAction)
        self.take_off_action_client.wait_for_server()
        self.land_client = actionlib.SimpleActionClient('Land', LandAction)
        self.land_client.wait_for_server()

    def handle_takeoff(self, takeoff):
        """ Sends takeoff action message to the drone action server
        @param takeoff:
        """
        goal = TakeOffGoal(takeoff=True)
        self.take_off_action_client.send_goal(goal)
        self.take_off_action_client.wait_for_result()
        result = self.take_off_action_client.get_result()

        if result.success:
            rospy.loginfo("Drone successfully took off")
            takeoff = True
        else:
            rospy.loginfo("Drone takeoff failed")

    def handle_land(self, land):
        """ Sends landing action message to the drone action server

        @param land:
        """
        goal = LandGoal(land=True)
        self.land_client.send_goal(goal)
        self.land_client.wait_for_result()
        result = self.land_client.get_result()

        if result.success:
            rospy.loginfo("Drone successfully landed")
            land = True
        else:
            rospy.loginfo("Drone landing failed")
