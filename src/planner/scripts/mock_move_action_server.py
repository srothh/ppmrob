#!/usr/bin/env python3

import rospy
import control.msg  # The action specification generates several messages for sending goals, receiving feedback, etc... This line imports the generated messages.
import common.config.defaults as defaults
import py_trees_ros


def cb_inside_exec():
    print("Executing move...")


def cb_after_goal_received(goal):
    print("Goal received: {goal}".format(goal=goal.target))


if __name__ == "__main__":
    rospy.init_node("mock_move_action_server")
    mock_as = py_trees_ros.mock.action_server.ActionServer(
        action_name=defaults.Control.MOVE_ACTION_NAMESPACE,
        action_type=control.msg.PlanningMoveAction,
        worker=cb_inside_exec,
        goal_received_callback=cb_after_goal_received,
        duration=1.0,
    )
    mock_as.start()
    rospy.spin()
