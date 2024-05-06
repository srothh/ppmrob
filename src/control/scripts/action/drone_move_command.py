#!/usr/bin/env python3

import rospy
import actionlib
from geometry_msgs.msg import Transform, Vector3, Quaternion
#from control.msg import MoveAction, MoveResult, MoveFeedback
#from drone.msg import MoveAction, MoveResult, MoveFeedback
from drone.msg import MoveAction, MoveResult, MoveFeedback, MoveGoal, ControlTransformActionGoal


class DroneMoveCommand(object):

    def __init__(self):
        # Drone move client
        self.drone_move_client = actionlib.SimpleActionClient("move", MoveAction)

        self.move_pub = rospy.Publisher('/TransformActionServer/goal', ControlTransformActionGoal, queue_size=10)

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

        rospy.loginfo("Move-command: x: %.2f y: %.2f z: %.2f r: %.2f " % (
            x, y, z, r))

        goal = MoveGoal(target=Transform(Vector3(x, y, z), Quaternion(0, 0, r, 0))))
        self.drone_move_client.send_goal(goal)
        #self.drone_move_client.wait_for_server()

        # For publishing as topic
        rospy.loginfo("Control: Publishing Transform Action Goal: x: %.2f y: %.2f z: %.2f r: %.2f " % (
            x, y, z, r))
        #self.move_pub.publish(goal)

        # self.drone_move_client.send_goal_and_wait(
        #     drone.msg.MoveActionGoal(target=Transform(Vector3(x, y, z), Quaternion(0, 0, r, 0))))
        result = self.drone_move_client.get_result()
        return result


