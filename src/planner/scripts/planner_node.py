#!/usr/bin/env python3

import py_trees
import py_trees_ros
import rospy
import planner.msg


class TelloCommands:
    TAKEOFF = "takeoff"
    LAND = "land"
    STOP = "stop"


"""
In py_trees behaviours are the leaf nodes. They represent either a check or an action. Created by subclassing py_trees.behaviour.Behaviour.
"""


class Leaf(py_trees.behaviour.Behaviour):
    """
    Connects to a subprocess to initiate a goal, and monitors the progress of that goal at each tick until the goal is completed, at which time the behaviour itself returns with success or failure (depending on success or failure of the goal itself).

    Key point - this behaviour itself should not be doing any work!
    Keep the scope of a single behaviour tight and focused, deploy larger concepts as subtrees.
    """

    def __init__(self, action_spec, action_goal, action_ns, name="Leaf"):
        """
        1-time initialisation.
        Rule of thumb: only include the initialisation relevant for being able to insert this behaviour in a tree for offline rendering to dot graphs.
        Other one-time initialisation requirements should be met via the `setup()` method.
        Which means:
        - No hardware connections that may not be there, e.g. usb lidars
        - No middleware connections to other software that may not be there, e.g. ROS pubs/subs/services
        - No need to fire up other needlessly heavy resources, e.g. heavy threads in the background
        """
        super(Leaf, self).__init__(name)
        self.action_spec = action_spec
        self.action_goal = action_goal
        self.action_ns = action_ns
        self.logger.debug(
            "  %s [%s::__init__()]" % (self.name, self.__class__.__name__)
        )

    def setup(self, timeout=15):
        """
        Called either manually in program or indirectly called by a parent behaviour when it's own setup method has been called.

        Delayed one-time initialisation that would otherwise interfere with offline rendering of this behaviour in a tree to dot graph, e.g.:
        - HW or driver initialisation
        - Middleware initialisation (e.g. ROS pubs/subs/services)
        """
        self.logger.debug("  %s [%s::setup()]" % (self.name, self.__class__.__name__))
        self.action_client = py_trees_ros.actions.ActionClient(
            action_spec=self.action_spec,
            action_goal=self.action_goal,
            action_namespace=self.action_ns,
        )
        # TODO replace with py-trees-ros action client (above )
        # FIXME this means that there exists Action.action in some package (import with 'from pkg.msg import Action, Goal')
        # self.action_client = actionlib.SimpleActionClient(action_server, Leaf)
        # see http://wiki.ros.org/actionlib#Using_the_ActionClient 6.2!
        return True

    def initialise(self):
        """
        Called first time a behaviour is ticked and anytime the status is not `RUNNING` thereafter.

        Do here any initialisation that is needed before putting the behaviour to work - configure and reset the behaviour ready for (repeated) execution.

        If you have vital initialisation in `setup()` ==> put a guard in here to barf the first time your behaviour is ticked if setup has not been called/succeeded!
        """
        self.logger.debug(
            "  %s [%s::initialise()]" % (self.name, self.__class__.__name__)
        )
        # # TODO ?
        # self.action_client.wait_for_server()
        # self.action_client.send_goal(self.action_goal)

    def update(self):
        """
        Called each time the behaviour is ticked.

        Work done here:
        - Triggering, checking, monitoring, ... anything not blocking!
            - never blocks, at most it just monitors the progress and holds up any decision making required by a tree that is ticking the behaviour by setting it’s status to RUNNING.
        - Set a feedback message
        - return a `py_trees.Status.[RUNNING, SUCCESS, FAILURE]`

        A behaviour has a naturally built in feedback message that can be cleared in the `initialise()` or `terminate()` methods and updated in the `update()` method. Alter a feedback message when significant events occur - e.g., victim found?
        """
        self.logger.debug("  %s [%s::update()]" % (self.name, self.__class__.__name__))
        """  
        Preempted - Processing of the goal was canceled by either another goal, or a cancel request sent to the action server

        Aborted - The goal was terminated by the action server without an external request from the action client to cancel
        """
        return self.action_client.update()

    def terminate(self, new_status):
        """
        Called whenever behavior switches to a non-running state.
        - SUCCESS || FAILURE : your behaviour's work cycle has finished
        - INVALID : a higher priority branch has interrupted, or shutting down

        Nothing to clean up in this example?
        """
        self.logger.debug(
            "  %s [%s::terminate()][%s->%s]"
            % (self.name, self.__class__.__name__, self.status, new_status)
        )


def create_root():
    # TODO build the final tree as if executing tree traversal algorithm: from root to bottom left to bottom right
    root = py_trees.composites.Sequence(name="Search & rescue N victims")
    rescue_subtree = py_trees.composites.Sequence(name="Rescue victim")
    stop_above_victim = py_trees_ros.actions.ActionClient(
        name="Stop above victim",
        action_spec=planner.msg.CommandAction,
        action_goal=planner.msg.CommandGoal(command=TelloCommands.STOP),
        action_namespace="command",
    )
    land_where_victim_found = py_trees_ros.actions.ActionClient(
        name="Land where victim found",
        action_spec=planner.msg.CommandAction,
        action_goal=planner.msg.CommandGoal(command=TelloCommands.LAND),
        action_namespace="command",
    )
    # takeoff_from_there = Leaf(planner.msg.LaunchAction, planner.msg.LaunchActionGoal(order=True), "launch")
    takeoff_from_there = py_trees_ros.actions.ActionClient(
        name="Takeoff from there",
        action_spec=planner.msg.CommandAction,
        action_goal=planner.msg.CommandGoal(command=TelloCommands.TAKEOFF),
        action_namespace="command",
    )

    root.add_children([rescue_subtree])
    rescue_subtree.add_children(
        [stop_above_victim, land_where_victim_found, takeoff_from_there]
    )

    return root


def setup_bt():
    """
    Set up the behaviour tree for the planning module.

    This function creates the root node of the behaviour tree and sets up the tree with a timeout of 15 seconds.

    Returns:
        py_trees_ros.trees.BehaviourTree: The configured behaviour tree.
    """
    root = create_root()
    tree = py_trees_ros.trees.BehaviourTree(root)
    # TODO is 15 seconds enough? what setup is needed anyway - the action servers?
    tree.setup(timeout=15)
    return tree


def run_bt(behavior_tree: py_trees_ros.trees.BehaviourTree, rate_hz=2):
    """
    Run the behavior tree.

    Args:
        behavior_tree (py_trees_ros.trees.BehaviourTree): The behavior tree to run.
        rate (float, optional): The rate at which the behavior tree is executed. Defaults to 2 Hz.

    Returns:
        None
    """
    condition_met = False  # FIXME: Replace with checking whether N victims have already been rescued - maybe use Blackboard for this? or ROS params?
    # TODO what about battery status? - add as the subtree with highest prio (most left) similar to the tut from py_trees_ros
    rate = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        if condition_met:  # FIXME found N victims? Blackboard?
            break
        behavior_tree.tick(
            pre_tick_handler=None, post_tick_handler=None
        )  # TODO add handlers?
        """
        When a behaviour tree ticks, it traverses the behaviours (starting at the root of the tree), ticking each behaviour, catching its result and then using that result to make decisions on the direction the tree traversal will take. This is the decision part of the tree. Once the traversal ends back at the root, the tick is over.

        Any blocking work should be happening somewhere else with a behaviour simply in charge of starting/monitoring and catching the result of that work.
        """
        py_trees.display.print_ascii_tree(tree.root, show_status=True)
        rate.sleep()


if __name__ == "__main__":
    try:
        # register the node with roscore, allowing it to communicate with other nodes
        rospy.init_node("planner")
        tree = setup_bt()
        run_bt(tree)
    except rospy.ROSInterruptException:
        pass
