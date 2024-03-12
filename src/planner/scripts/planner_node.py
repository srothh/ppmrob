#!/usr/bin/env python3
import functools

import actionlib
import py_trees
import py_trees_ros
import rospy

# import control.msg
import test_planning.msg

# import grid_based_sweep
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point

import common.config.defaults as defaults  # TODO add to dockerfile as per issue!


BB_VAR_RETURNED_HOME = "returned_home"
BB_VAR_HOME_COORDINATES = "home_coordinates"
BB_VAR_VICTIM_FOUND = "victim_found"


class CreatePlan(py_trees.behaviour.Behaviour):
    def __init__(self, name, action_server_name):
        super(CreatePlan, self).__init__(name)
        self.map_subscriber = rospy.Subscriber(
            "/mapping/map", String, self.map_callback
        )
        self.map_data = None
        self.action_client = actionlib.SimpleActionClient(
            action_server_name, planner.msg.PlanAction
        )
        self.action_client.wait_for_server()

    def map_callback(self, msg):
        self.map_data = msg

    def initialise(self):
        pass

    def update(self):
        if self.map_data is None:
            rospy.loginfo("Waiting for map data...")
            return py_trees.common.Status.RUNNING

        # plan = grid_based_sweep.generate_plan()
        goal = planner.msg.PlanGoal()
        # goal.x = plan[0]
        # goal.y = plan[1]

        self.action_client.send_goal(goal)
        self.action_client.wait_for_result()

        result = self.action_client.get_result()
        print(result)

        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.map_subscriber.unregister()


"""
In py_trees behaviours are the leaf nodes. They represent either a check or an action. Created by subclassing py_trees.behaviour.Behaviour.
"""


class Leaf(py_trees.behaviour.Behaviour):
    """
    Connects to a subprocess to initiate a goal, and monitors the progress of that goal at each tick until the goal is completed, at which time the behaviour itself returns with success or failure (depending on success or failure of the goal itself).
    Key point - this behaviour itself should not be doing any work!
    Keep the scope of a single behaviour tight and focused, deploy larger concepts as subtrees.
    """

    def __init__(self, name="Leaf"):
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
        self.logger.debug(
            "  %s [%s::__init__()]" % (self.name, self.__class__.__name__)
        )

    def setup(self, timeout=15):
        """
        Called either manually in program or indirectly called by a parent behaviour when its own setup method has been called.
        Delayed one-time initialisation that would otherwise interfere with offline rendering of this behaviour in a tree to dot graph, e.g.:
        - HW or driver initialisation
        - Middleware initialisation (e.g. ROS pubs/subs/services)
        """
        self.logger.debug("  %s [%s::setup()]" % (self.name, self.__class__.__name__))
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

    def update(self):
        """
        Called each time the behaviour is ticked.
        Work done here:
        - Triggering, checking, monitoring, ... anything not blocking!
            - never blocks, at most it just monitors the progress and holds up any decision making required by a tree that is ticking the behaviour by setting its status to RUNNING.
        - Set a feedback message
        - return a `py_trees.Status.[RUNNING, SUCCESS, FAILURE]`
        A behaviour has a naturally built in feedback message that can be cleared in the `initialise()` or `terminate()` methods and updated in the `update()` method. Alter a feedback message when significant events occur - e.g., victim found?
        """
        self.logger.debug("  %s [%s::update()]" % (self.name, self.__class__.__name__))
        """  
        Preempted - Processing of the goal was canceled by either another goal, or a cancel request sent to the action server
        Aborted - The goal was terminated by the action server without an external request from the action client to cancel
        """
        return py_trees.common.Status.SUCCESS

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


class MyDynamicActionClient(py_trees_ros.actions.ActionClient):
    def initialise(self):
        home_coords = py_trees.blackboard.Blackboard().get(BB_VAR_HOME_COORDINATES)
        self.action_goal = test_planning.msg.PlanningMoveGoal(
            x=home_coords.x,  # FIXME
            y=home_coords.y,  # FIXME
        )
        super().initialise()

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.SUCCESS:
            py_trees.blackboard.Blackboard().set(
                BB_VAR_RETURNED_HOME, True
            )  # TODO as constant
        super().terminate(new_status)


class PlanningMoveDynamicActionClient(py_trees_ros.actions.ActionClient):

    def initialise(self):
        # TODO as constant
        planned_path = py_trees.blackboard.Blackboard().get("planned_path")
        # TODO check that this works as intended!
        self.action_goal = test_planning.msg.PlanningMoveGoal(target=planned_path)
        super().initialise()


def create_root():
    # TODO build the final tree as if executing tree traversal algorithm: from root to bottom left to bottom right
    # nodes
    root = py_trees.composites.Parallel("Mission")
    topics2bb = py_trees.composites.Sequence("Topics2BB")
    # TODO change the topic name and adjust our architecture to use this! - just ME
    # TODO use the constants from common folder for: threshold, topic name
    # see tut for on which topic to publish to and which msg (only percentage)
    battery2bb = py_trees_ros.battery.ToBlackboard(
        name="Battery2BB", topic_name="/battery/state", threshold=10.0
    )
    # TODO replace with the actual topic name - constant from common folder!
    # FIXME maybe just delete and create own behavior that will just use service to get home coordinates?
    home_coords2bb = py_trees_ros.subscribers.ToBlackboard(
        name="HomeCoords2BB",
        topic_name=defaults.Odometry.HOME_COORDS_TOPIC_NAME,
        topic_type=Point,
        blackboard_variables={BB_VAR_HOME_COORDINATES: None},
    )
    victim_found2bb = py_trees_ros.subscribers.ToBlackboard(
        name="VictimFound2BB",
        topic_name=defaults.Mapping.VICTIM_FOUND_TOPIC_NAME,
        topic_type=Bool,
        # get rid of the annoying sub-data field
        blackboard_variables={BB_VAR_VICTIM_FOUND: "data"},
    )
    priorities = py_trees.composites.Selector("Priorities")
    battery_check = py_trees.composites.Sequence("Battery check")
    is_battery_low = py_trees.blackboard.CheckBlackboardVariable(
        name="Battery low?",
        variable_name="battery_low_warning",
        expected_value=True,
    )
    # FIXME in this case ask mapping for home coordinates and send them per move action to control node!
    # FIXME I just need this actionclient to set variable in blackboard, waypoint needs to be set just once!
    return_home = MyDynamicActionClient(
        name="Return home",
        action_spec=test_planning.msg.PlanningMoveAction,
        action_namespace=defaults.Planning.MOVE_ACTION_NAMESPACE,
    )
    search_and_rescue = py_trees.composites.Sequence(name="Search & rescue victim")
    search_subtree = py_trees.composites.Sequence(name="Search victim")
    search_subtree_condition = py_trees.decorators.Condition(
        child=search_subtree, status=py_trees.common.Status.FAILURE
    )
    takeoff = py_trees_ros.actions.ActionClient(
        name="Takeoff",
        action_spec=test_planning.msg.PlanningCommandAction,
        action_goal=test_planning.msg.PlanningCommandGoal(
            command=defaults.TelloCommands.TAKEOFF
        ),
        action_namespace=defaults.Planning.COMMAND_ACTION_NAMESPACE,
    )
    is_victim_found = py_trees.blackboard.CheckBlackboardVariable(
        name="Victim found?",
        variable_name=BB_VAR_VICTIM_FOUND,
        expected_value=True,
    )
    is_victim_found_inverter = py_trees.decorators.Inverter(child=is_victim_found)
    path_planning = py_trees.behaviours.Success("Path planning")
    # FIXME following needs output of path planning! -> make dynamic action client
    move_to_next_position = PlanningMoveDynamicActionClient(
        name="Move to next position",
        action_spec=test_planning.msg.PlanningMoveAction,
        action_namespace=defaults.Planning.MOVE_ACTION_NAMESPACE,
    )
    rescue_subtree = py_trees.composites.Sequence(name="Rescue victim")
    stop_above_victim = py_trees_ros.actions.ActionClient(
        name="Stop above victim",
        action_spec=test_planning.msg.PlanningCommandAction,
        action_goal=test_planning.msg.PlanningCommandGoal(
            command=defaults.TelloCommands.STOP
        ),
        action_namespace=defaults.Planning.COMMAND_ACTION_NAMESPACE,
    )
    land_where_victim_found = py_trees_ros.actions.ActionClient(
        name="Land where victim found",
        action_spec=test_planning.msg.PlanningCommandAction,
        action_goal=test_planning.msg.PlanningCommandGoal(
            command=defaults.TelloCommands.LAND
        ),
        action_namespace=defaults.Planning.COMMAND_ACTION_NAMESPACE,
    )
    # takeoff_from_there = Leaf(planner.msg.LaunchAction, planner.msg.LaunchActionGoal(order=True), "launch")
    # tree
    root.add_children([topics2bb, priorities])
    topics2bb.add_children([battery2bb, home_coords2bb, victim_found2bb])
    priorities.add_children([battery_check, search_and_rescue])
    battery_check.add_children([is_battery_low, return_home])
    search_and_rescue.add_children([takeoff, search_subtree_condition, rescue_subtree])
    search_subtree.add_children(
        [
            is_victim_found_inverter,
            path_planning,
            move_to_next_position,
        ]
    )
    rescue_subtree.add_children([stop_above_victim, land_where_victim_found])
    return root


def post_tick_handler(snapshot_visitor, tree):
    # print(py_trees.display.ascii_tree(tree.root, snapshot_information=snapshot_visitor))
    # the following actually shows more info than the above
    py_trees.display.print_ascii_tree(tree.root, show_status=True)


def setup_bt(timeout=15):
    """
    Set up the behaviour tree for the planning module.
    This function creates the root node of the behaviour tree and sets up the tree with a given timeout.
    Args:
        timeout (int, optional): The timeout for setting up the behaviour tree. Defaults to 15.
    Returns:
        py_trees_ros.trees.BehaviourTree: The configured behaviour tree.
    """
    root = create_root()
    tree = py_trees_ros.trees.BehaviourTree(root)
    # `SnapshotVisitor` collects runtime data to be used by visualisations
    snapshot_visitor = py_trees.visitors.SnapshotVisitor()
    # add a post-tick handler to print the tree after each tick in the console
    tree.add_post_tick_handler(functools.partial(post_tick_handler, snapshot_visitor))
    # `DebugVisitor` prints debug logging messages to stdout
    tree.visitors.append(py_trees.visitors.DebugVisitor())
    tree.visitors.append(snapshot_visitor)
    # TODO is 15 seconds enough? what setup is needed anyway - just the action servers in control package?
    tree.setup(timeout=timeout)
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
    rate = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        # TODO not sure if this makes sense - maybe just sweep whole area until battery runs out
        # if (
        #     py_trees.blackboard.Blackboard().get("number of victims found and rescued")
        #     == num_of_victims
        # ):
        #     break
        behavior_tree.tick()
        """
        When a behaviour tree ticks, it traverses the behaviours (starting at the root of the tree), ticking each behaviour, catching its result and then using that result to make decisions on the direction the tree traversal will take. This is the decision part of the tree. Once the traversal ends back at the root, the tick is over.
        Any blocking work should be happening somewhere else with a behaviour simply in charge of starting/monitoring and catching the result of that work.
        """
        if py_trees.blackboard.Blackboard().get(BB_VAR_RETURNED_HOME):
            break
        rate.sleep()


if __name__ == "__main__":
    try:
        # register the node with roscore, allowing it to communicate with other nodes
        rospy.init_node("planner")
        tree = setup_bt()
        py_trees.display.render_dot_tree(tree.root, name="planner_tree")
        # run_bt(tree)
    except rospy.ROSInterruptException:
        pass
