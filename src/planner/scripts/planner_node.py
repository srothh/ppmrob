#!/usr/bin/env python3

import functools
import actionlib
import py_trees
import py_trees_ros
import rospy
from geometry_msgs.msg import Point

# import grid_based_sweep
from std_msgs.msg import Bool, String

import common.config.defaults as defaults
import control.msg

BB_VAR_RETURNED_HOME = "returned_home"
BB_VAR_HOME_COORDINATES = "home_coordinates"
BB_VAR_VICTIM_FOUND = "victim_found"
BB_VAR_WAYPOINT = "waypoint"
BB_VAR_NUM_OF_RESCUED_VICTIMS = "num_of_rescued_victims"


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


class Leaf(py_trees.Behaviour):
    """
    Connects to a subprocess to initiate a goal, and monitors the progress of that goal at each tick until the goal is completed, at which time the behaviour itself returns with success or failure (depending on success or failure of the goal itself).
    Key point - this behaviour itself should not be doing any work!
    Keep the scope of a single behaviour tight and focused, deploy larger concepts as subtrees.
    """

    def __init__(self, name="Leaf"):
        super(Leaf, self).__init__(name)
        self.logger.debug(
            "  %s [%s::__init__()]" % (self.name, self.__class__.__name__)
        )

    def setup(self, timeout=15):
        self.logger.debug("  %s [%s::setup()]" % (self.name, self.__class__.__name__))
        return True

    def initialise(self):
        self.logger.debug(
            "  %s [%s::initialise()]" % (self.name, self.__class__.__name__)
        )

    def update(self):
        self.logger.debug("  %s [%s::update()]" % (self.name, self.__class__.__name__))
        """  
        Preempted - Processing of the goal was canceled by either another goal, or a cancel request sent to the action server
        Aborted - The goal was terminated by the action server without an external request from the action client to cancel
        """
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.logger.debug(
            "  %s [%s::terminate()][%s->%s]"
            % (self.name, self.__class__.__name__, self.status, new_status)
        )


class PlanningMoveDynamicActionClient(py_trees_ros.actions.ActionClient):
    def initialise(self):
        planned_path = py_trees.blackboard.Blackboard().get(BB_VAR_WAYPOINT)
        rospy.loginfo(f"Planned path: {planned_path}")
        self.action_goal = control.msg.PlanningMoveGoal(target=planned_path)
        super().initialise()


class IncrementBbVar(py_trees.behaviours.Success):
    """
    Custom behaviour to increment a blackboard variable.
    """

    def __init__(self, name, variable_name):
        super(IncrementBbVar, self).__init__(name)
        self.variable_name = variable_name

    @py_trees.utilities.static_variables(counter=0)
    def initialise(self):
        self.blackboard = py_trees.blackboard.Blackboard()
        IncrementBbVar.initialise.counter += 1
        self.blackboard.set(
            self.variable_name, IncrementBbVar.initialise.counter, overwrite=True
        )
        rospy.loginfo(f"Number of rescued victims: {IncrementBbVar.initialise.counter}")
        # print(self.blackboard)


def create_root():
    """
    Creates the behaviour tree and returns the root node.

    The tree is built as if executing a tree traversal algorithm: from root to bottom left to bottom right nodes.
    """
    # nodes
    root = py_trees.composites.Parallel("Mission")
    topics2bb = py_trees.composites.Sequence("Topics2BB")
    # preferred way of getting data from a topic according to docs (instead of, e.g., `py_trees_ros.subscribers.CheckData`)
    victim_found2bb = py_trees_ros.subscribers.ToBlackboard(
        name="VictimFound2BB",
        topic_name=defaults.Mapping.VICTIM_FOUND_TOPIC_NAME,
        topic_type=Bool,
        # get rid of the annoying sub-data field
        blackboard_variables={BB_VAR_VICTIM_FOUND: "data"},
        initialise_variables={BB_VAR_VICTIM_FOUND: False},
    )
    battery2bb = py_trees_ros.battery.ToBlackboard(
        name="Battery2BB",
        topic_name=defaults.drone_battery_sensor_publish_topic_name,
        threshold=defaults.Drone.BATTERY_THRESHOLD,
    )
    priorities = py_trees.composites.Selector("Priorities")
    battery_check = py_trees.composites.Sequence("Battery check")
    is_battery_low = py_trees.blackboard.CheckBlackboardVariable(
        name="Battery low?",
        variable_name="battery_low_warning",
        expected_value=True,
    )
    return_home = py_trees.composites.Sequence("Return home")
    fly_home = py_trees_ros.actions.ActionClient(
        name="Fly home",
        action_spec=control.msg.PlanningMoveAction,
        action_goal=control.msg.PlanningMoveGoal(
            target=Point(x=0.0, y=0.0, z=0.0)  # home coordinates
        ),
        action_namespace=defaults.Control.MOVE_ACTION_NAMESPACE,
        override_feedback_message_on_running="Returning home...",
    )
    land_home = py_trees_ros.actions.ActionClient(
        name="Land home",
        action_spec=control.msg.PlanningCommandAction,
        action_goal=control.msg.PlanningCommandGoal(
            command=defaults.TelloCommands.LAND
        ),
        action_namespace=defaults.Control.COMMAND_ACTION_NAMESPACE,
        override_feedback_message_on_running="Landing...",
    )
    terminate = py_trees.blackboard.SetBlackboardVariable(
        name="Terminate", variable_name=BB_VAR_RETURNED_HOME, variable_value=True
    )
    search_and_rescue = py_trees.composites.Sequence(name="Search & rescue victim")
    takeoff = py_trees_ros.actions.ActionClient(
        name="Takeoff",
        action_spec=control.msg.PlanningCommandAction,
        action_goal=control.msg.PlanningCommandGoal(
            command=defaults.TelloCommands.TAKEOFF
        ),
        action_namespace=defaults.Control.COMMAND_ACTION_NAMESPACE,
        override_feedback_message_on_running="Taking off...",
    )
    search_subtree = py_trees.composites.Sequence(name="Search victim")
    search_subtree_condition = py_trees.decorators.Condition(
        child=search_subtree, status=py_trees.common.Status.FAILURE
    )
    is_victim_found = py_trees.blackboard.CheckBlackboardVariable(
        name="Victim found?",
        variable_name=BB_VAR_VICTIM_FOUND,
        expected_value=True,
    )
    is_victim_found_inverter = py_trees.decorators.Inverter(child=is_victim_found)
    path_planning = py_trees.behaviours.Success("Path planning")
    move_to_next_position = PlanningMoveDynamicActionClient(
        name="Move to next position",
        action_spec=control.msg.PlanningMoveAction,
        action_namespace=defaults.Control.MOVE_ACTION_NAMESPACE,
        override_feedback_message_on_running="Moving to next position...",
    )
    rescue_subtree = py_trees.composites.Sequence(name="Rescue victim")
    land_where_victim_found = py_trees_ros.actions.ActionClient(
        name="Land where victim found",
        action_spec=control.msg.PlanningCommandAction,
        action_goal=control.msg.PlanningCommandGoal(
            command=defaults.TelloCommands.LAND
        ),
        action_namespace=defaults.Control.COMMAND_ACTION_NAMESPACE,
        override_feedback_message_on_running="Landing where victim found...",
    )
    victim_rescued = IncrementBbVar("Rescued victim", BB_VAR_NUM_OF_RESCUED_VICTIMS)
    # tree
    root.add_children([topics2bb, priorities])
    topics2bb.add_children([victim_found2bb, battery2bb])
    priorities.add_children([battery_check, search_and_rescue])
    battery_check.add_children([is_battery_low, return_home])
    return_home.add_children([fly_home, land_home, terminate])
    search_and_rescue.add_children([takeoff, search_subtree_condition, rescue_subtree])
    search_subtree.add_children(
        [
            is_victim_found_inverter,
            path_planning,
            move_to_next_position,
        ]
    )
    rescue_subtree.add_children([land_where_victim_found, victim_rescued])
    return root


def post_tick_handler(snapshot_visitor, tree):
    # print(py_trees.display.ascii_tree(tree.root, snapshot_information=snapshot_visitor))
    # the following actually shows more info than the above
    # py_trees.display.print_ascii_tree(tree.root, show_status=True)
    pass


def setup_bt(timeout=defaults.Planning.BT_SETUP_TIMEOUT):
    """
    Set up the behaviour tree.
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
    """
    Fetch the private parameter from the parameter server.
    KeyError is raised if the parameter is not set (see launch file for how to run).
    """
    num_of_victims_to_rescue = rospy.get_param("~num_of_victims_to_rescue")
    rospy.loginfo(f"Number of victims to rescue: {num_of_victims_to_rescue}")
    # py_trees.blackboard.Blackboard().set(BB_VAR_NUM_OF_RESCUED_VICTIMS, 0)
    # py_trees.blackboard.Blackboard().set(BB_VAR_RETURNED_HOME, False)
    while not rospy.is_shutdown():
        if (
            py_trees.blackboard.Blackboard().get(BB_VAR_RETURNED_HOME)
            or py_trees.blackboard.Blackboard().get(BB_VAR_NUM_OF_RESCUED_VICTIMS)
            == num_of_victims_to_rescue
        ):
            rospy.loginfo("Mission completed.")
            break
        """
        When a behaviour tree ticks, it traverses the behaviours (starting at the root of the tree), ticking each behaviour, catching its result and then using that result to make decisions on the direction the tree traversal will take. This is the decision part of the tree. Once the traversal ends back at the root, the tick is over.
        Any blocking work should be happening somewhere else with a behaviour simply in charge of starting/monitoring and catching the result of that work.
        """
        behavior_tree.tick()
        rate.sleep()


if __name__ == "__main__":
    try:
        # register the node with roscore, allowing it to communicate with other nodes
        rospy.init_node("planner")
        # for testing purpose
        # py_trees.logging.level = py_trees.logging.Level.DEBUG
        tree = setup_bt()
        # py_trees.display.render_dot_tree(tree.root, name="planner_tree")
        run_bt(tree)
    except rospy.ROSInterruptException:
        pass
