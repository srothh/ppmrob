#!/usr/bin/env python3

import rospy
import py_trees # TODO add to dependencies!

def create_behavior_tree():

    root = py_trees.composites.Sequence("Rescuer drone")

    rescue_loop = py_trees.composites.Sequence("Rescue loop")
    rescue_runner = py_trees.decorators.FailureIsRunning(child=rescue_loop)

    go_home = py_trees.composites.Sequence("Return home subroutine")
    fly_home = py_trees.behaviours.Success("Navigate to home coordinates")
    stop_and_land = py_trees.behaviours.Success("Stop and Land")
    go_home.add_children([fly_home, stop_and_land])

    rescue = py_trees.composites.Sequence("Rescue subroutine")
    repeater = FailUntil("Repeat N times", cap=3)

    identify_next = py_trees.behaviours.Success("Identify next cell")

    cell_processor = py_trees.composites.Sequence("Process current cell")

    add_to_mapping = py_trees.behaviours.Success("Add cell to mapping")
    classification = py_trees.behaviours.Success("Victim classification")

    cell_processor.add_children([add_to_mapping, classification])

    exploration_sequence = py_trees.composites.Sequence("Exploration", children=[identify_next, cell_processor])

    until_fail = py_trees.decorators.FailureIsRunning(child=exploration_sequence, name="Repeat until fail")

    rescue_maneuver = py_trees.composites.Sequence("Rescue maneuver")

    takeoff = py_trees.behaviours.Success("Take off")

    rescue_maneuver.add_children([stop_and_land, takeoff])
    rescue.add_children([until_fail, rescue_maneuver])
    rescue_loop.add_children([rescue, repeater])
    root.add_children([rescue_runner, go_home])

    return root



if __name__ == '__main__':
    try:
        rospy.init_node('planing')  # register the node with roscore, allowing it to communicate with other nodes
        behavior_tree = create_behavior_tree()
        # TODO go over the tree etc. (define class for that I guess?)
    except rospy.ROSInterruptException:
        pass