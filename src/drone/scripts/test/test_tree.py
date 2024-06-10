#!/usr/bin/env python3

import py_trees
import py_trees_ros

def create_root():
    """
    Create a basic tree and start a 'Topics2BB' work sequence that
    takes the asynchronicity out of subscription.

    Returns:
        :class:~py_trees.behaviour.Behaviour: the root of the tree
    """

    writer = py_trees.blackboard.Client(name="Writer")
    writer.register_key(key="nested", access=py_trees.common.Access.WRITE)
    reader = py_trees.blackboard.Client(name="Reader")
    reader.register_key(key="nested", access=py_trees.common.Access.READ)


    root = py_trees.composites.Parallel("Tutorial")

    topics2bb = py_trees.composites.Sequence("Topics2BB")
    battery2bb = py_trees_ros.battery.ToBlackboard(name="Battery2BB",
                                                   topic_name="/battery/state",
                                                   threshold=30.0
                                                   )
    priorities = py_trees.composites.Selector("Priorities")
    idle = py_trees.behaviours.Running(name="Idle")

    root.add_child(topics2bb)
    topics2bb.add_child(battery2bb)
    root.add_child(priorities)
    priorities.add_child(idle)
    return root

if __name__ == 'main':
    print(create_root())