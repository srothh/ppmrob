#!/usr/bin/env python3

import rospy
import actionlib
from geometry_msgs.msg import Transform, Vector3, Quaternion
#from control.msg import MoveAction, MoveResult, MoveFeedback
#from drone.msg import MoveAction, MoveResult, MoveFeedback
from drone.msg import MoveAction, MoveResult, MoveFeedback


class DroneMoveCommand(object):

    def __init__(self):
        # Drone move client
        self.drone_move_client = actionlib.SimpleActionClient("Move", MoveAction)

        self.drone_move_client.wait_for_server()

        rospy.loginfo("drone move client initalized")


    def feedback_callback(self, feedback):

        rospy.loginfo("Received feedback from drone-node: " + feedback)

    def move_drone(self, x, y, z, r):
        """ Function which sends move command to the drone node with given parameters

        @param x:
        @param y:
        @param z:
        @param r:
        @return:
        """
        self.drone_move_client.send_goal_and_wait(
            drone.msg.MoveGoal(target=Transform(Vector3(x, y, z), Quaternion(0, 0, r, 0))))
        result = self.drone_move_client.get_result()
        return result


