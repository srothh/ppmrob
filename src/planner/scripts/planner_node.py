#!/usr/bin/env python3

from collections import deque
import functools
import json
import os
from typing import List

import numpy
from numpy import array

import actionlib
import py_trees
import py_trees_ros
import rospy

from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid
import pathfinding
import common.config.defaults as defaults
import control.msg

DRONE_MOVEMENT_INCREMENT = 30

BB_VAR_RETURNED_HOME = "returned_home"
BB_VAR_VICTIM_FOUND = "victim_found"
BB_VAR_WAYPOINT = "waypoint"
BB_VAR_NUM_OF_RESCUED_VICTIMS = "num_of_rescued_victims"

BB_VAR_WAYPOINTS = "waypoints"
BB_VAR_MAP_WIDTH = "map_width"
BB_VAR_MAP_DATA = "map_data"
BB_VAR_WORLD_POS = "world_pos"


class DynamicPlan(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(DynamicPlan, self).__init__(name)

    def update(self):
        path = dynamic_plan()
        py_trees.blackboard.Blackboard().plan = path
        return (
            py_trees.common.Status.FAILURE
            if len(path) == 0
            else py_trees.common.Status.SUCCESS
        )


class UnexploredPlan(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(UnexploredPlan, self).__init__(name)

    def update(self):
        path = path_to_unexplored()
        py_trees.blackboard.Blackboard().plan = path
        return (
            py_trees.common.Status.FAILURE
            if len(path) == 0
            else py_trees.common.Status.SUCCESS
        )


class HomePlan(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(HomePlan, self).__init__(name)

    def update(self):
        path = path_home()
        py_trees.blackboard.Blackboard().plan = path
        return (
            py_trees.common.Status.FAILURE
            if len(path) == 0
            else py_trees.common.Status.SUCCESS
        )


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


class ReturnHomeDynamicActionClient(py_trees_ros.actions.ActionClient):
    def initialise(self):
        planned_path = py_trees.blackboard.Blackboard().plan
        if len(planned_path) == 0:
            rospy.loginfo("Path could not be calculated")
        else:
            rospy.loginfo("Returning home...")
        self.action_goal = control.msg.PlanningMoveGoal(target=planned_path)
        super().initialise()


class PlanningMoveDynamicActionClient(py_trees_ros.actions.ActionClient):
    def initialise(self):
        planned_path = py_trees.blackboard.Blackboard().plan
         if len(planned_path) == 0:
            rospy.loginfo("Path could not be calculated")
        else:
            rospy.loginfo(f"Planned path: {planned_path}")
        self.action_goal = control.msg.PlanningMoveGoal(target=planned_path)
        super().initialise()


class PlanningMoveInteractiveActionClient(py_trees_ros.actions.ActionClient):

    def initialise(self):
        waypoint = py_trees.blackboard.Blackboard().waypoints.popleft()
        self.action_goal = control.msg.PlanningMoveGoal(target=[waypoint])
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
        blackboard = py_trees.blackboard.Blackboard()
        IncrementBbVar.initialise.counter += 1
        blackboard.set(
            self.variable_name, IncrementBbVar.initialise.counter, overwrite=True
        )
        rospy.loginfo(f"Number of rescued victims: {IncrementBbVar.initialise.counter}")


def dynamic_plan():
    if py_trees.blackboard.Blackboard().get(BB_VAR_MAP_WIDTH) == 0:
        return []
    grid = flat_to_2d(py_trees.blackboard.Blackboard().get(BB_VAR_MAP_DATA), py_trees.blackboard.Blackboard().get(BB_VAR_MAP_WIDTH))

    print(grid)

    world_pos = py_trees.blackboard.Blackboard().get(BB_VAR_WORLD_POS)

    print(world_pos)

    grid_pos_x, grid_pos_y = world_to_grid(world_pos.x,
                                           world_pos.z)

    print(grid_pos_x, grid_pos_y)
    path = []
    if grid[grid_pos_x][grid_pos_y + 1] == 50:
        path = [Point(world_pos.x + DRONE_MOVEMENT_INCREMENT, world_pos.y, world_pos.z)]
    elif grid[grid_pos_x + 1][grid_pos_y] == 50:
        path = [Point(world_pos.x, world_pos.y, world_pos.z + DRONE_MOVEMENT_INCREMENT)]
    elif grid[grid_pos_x][grid_pos_y - 1] == 50:
        path = [Point(world_pos.x - DRONE_MOVEMENT_INCREMENT, world_pos.y, world_pos.z)]
    elif grid[grid_pos_x - 1][grid_pos_y] == 50:
        path = [Point(world_pos.x, world_pos.y, world_pos.z - DRONE_MOVEMENT_INCREMENT)]
    return path


def path_to_unexplored():
    if py_trees.blackboard.Blackboard().get(BB_VAR_MAP_WIDTH) == 0:
        return []
    grid = flat_to_2d(py_trees.blackboard.Blackboard().get(BB_VAR_MAP_DATA), py_trees.blackboard.Blackboard().get(BB_VAR_MAP_WIDTH))
    for row in range(len(grid)):
        for cell in range(len(grid[row])):
            if grid[row][cell] == 50:
                return path_to_pos(row, cell)
    return []


def path_home():
    return path_to_pos(0, 0)


def path_to_pos(x, y):
    if py_trees.blackboard.Blackboard().get(BB_VAR_MAP_WIDTH) == 0:
        return []
    # print(x, " : ", y)
    grid = flat_to_2d(py_trees.blackboard.Blackboard().get(BB_VAR_MAP_DATA), py_trees.blackboard.Blackboard().get(BB_VAR_MAP_WIDTH))
    # print("---------")
    # print(grid)
    # print("---------")
    grid_pos_x, grid_pos_y = world_to_grid(py_trees.blackboard.Blackboard().get(BB_VAR_WORLD_POS).x,
                                           py_trees.blackboard.Blackboard().get(BB_VAR_WORLD_POS).z)
    current = (grid_pos_x, grid_pos_y)
    # print("current grid_pos: ",current)
    target = (x, y)
    path_indices = pathfinding.a_star(grid, current, target)
    # print("path: ", path_indices)
    path = []
    for point in path_indices:
        w_pos = grid_to_world(point[0], point[1])

        path.append(Point(w_pos[0], py_trees.blackboard.Blackboard().get(BB_VAR_WORLD_POS).y, w_pos[1]))

    # print("world_path: ",path)
    return path


def create_root_interactive():
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
        action_goal=control.msg.PlanningMoveGoal(target=[Point(x=0, y=0)]),
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
    move_to_next_position = PlanningMoveInteractiveActionClient(
        name="Move to next position",
        action_spec=control.msg.PlanningMoveAction,
        action_namespace=defaults.Control.MOVE_ACTION_NAMESPACE,
        override_feedback_message_on_running="Moving to next position...",
    )
    is_waypoints_empty = py_trees.meta.inverter(
        py_trees.blackboard.CheckBlackboardVariable
    )(
        name="All waypoints flown?",
        variable_name=BB_VAR_WAYPOINTS,
        expected_value=deque([]),
    )
    rescue_subtree = py_trees.composites.Sequence(name="Rescue victim")
    is_victim_actually_found = py_trees.blackboard.CheckBlackboardVariable(
        name="Victim actually found?",
        variable_name=BB_VAR_VICTIM_FOUND,
        expected_value=True,
    )
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
        [is_victim_found_inverter, move_to_next_position, is_waypoints_empty]
    )
    rescue_subtree.add_children(
        [is_victim_actually_found, land_where_victim_found, victim_rescued]
    )
    return root


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
        blackboard_variables={BB_VAR_VICTIM_FOUND: "data."},
        initialise_variables={BB_VAR_VICTIM_FOUND: False},
    )
    battery2bb = py_trees_ros.battery.ToBlackboard(
        name="Battery2BB",
        topic_name=defaults.drone_battery_sensor_publish_topic_name,
        threshold=defaults.Drone.BATTERY_THRESHOLD,
    )
    map2bb = py_trees_ros.subscribers.ToBlackboard(
        name="Map2BB",
        topic_name=defaults.Mapping.OCCUPANCY_GRID_TOPIC_NAME,
        topic_type=OccupancyGrid,
        blackboard_variables={BB_VAR_MAP_WIDTH: "info.width", BB_VAR_MAP_DATA: "data"},
        initialise_variables={BB_VAR_MAP_WIDTH: 0, BB_VAR_MAP_DATA: []}
    )
    world_pos2bb = py_trees_ros.subscribers.ToBlackboard(
        name="WorldPos2BB",
        topic_name=defaults.Control.WORLD_POSITION_TOPIC_NAME,
        topic_type=Point,
        blackboard_variables={BB_VAR_WORLD_POS: None},
        initialise_variables={BB_VAR_WORLD_POS: Point(0, 0, 0)}
    )


    priorities = py_trees.composites.Selector("Priorities")
    battery_check = py_trees.composites.Sequence("Battery check")
    is_battery_low = py_trees.blackboard.CheckBlackboardVariable(
        name="Battery low?",
        variable_name="battery_low_warning",
        expected_value=True,
    )
    return_home = py_trees.composites.Sequence("Return home")
    plan_home = HomePlan("Plan path home")
    fly_home = ReturnHomeDynamicActionClient(
        name="Fly home",
        action_spec=control.msg.PlanningMoveAction,
        action_namespace=defaults.Control.MOVE_ACTION_NAMESPACE,
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
    path_planning = py_trees.composites.Selector("Path planning")
    dynamic = DynamicPlan("Dynamic planning")
    unexplored = UnexploredPlan("Plan path to unexplored cell")
    move_to_next_position = PlanningMoveDynamicActionClient(
        name="Move to next position",
        action_spec=control.msg.PlanningMoveAction,
        action_namespace=defaults.Control.MOVE_ACTION_NAMESPACE,
        override_feedback_message_on_running="Moving to next position...",
    )
    rescue_subtree = py_trees.composites.Sequence(name="Rescue victim")
    is_victim_actually_found = py_trees.blackboard.CheckBlackboardVariable(
        name="Victim actually found?",
        variable_name=BB_VAR_VICTIM_FOUND,
        expected_value=True,
    )
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
    topics2bb.add_children([world_pos2bb, map2bb, victim_found2bb, battery2bb])
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
    path_planning.add_children([dynamic, unexplored, plan_home])
    rescue_subtree.add_children(
        [is_victim_actually_found, land_where_victim_found, victim_rescued]
    )
    return root


def post_tick_handler(snapshot_visitor, tree):
    # print(py_trees.display.ascii_tree(tree.root, snapshot_information=snapshot_visitor))
    # the following actually shows more info than the above
    py_trees.display.print_ascii_tree(tree.root, show_status=True)
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

    if rospy.get_param("~interactive_waypoints"):
        root = create_root_interactive()
    else:
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


def lead_drone_into_safe_state():
    ac_stop_cmd = actionlib.SimpleActionClient(
        defaults.Control.COMMAND_ACTION_NAMESPACE, control.msg.PlanningCommandAction
    )
    ac_stop_cmd.wait_for_server()
    ac_stop_cmd.send_goal(
        control.msg.PlanningCommandGoal(command=defaults.TelloCommands.STOP)
    )
    ac_stop_cmd.wait_for_result()
    ac_land_cmd = actionlib.SimpleActionClient(
        defaults.Control.COMMAND_ACTION_NAMESPACE, control.msg.PlanningCommandAction
    )
    ac_land_cmd.wait_for_server()
    ac_land_cmd.send_goal(
        control.msg.PlanningCommandGoal(command=defaults.TelloCommands.LAND)
    )
    ac_land_cmd.wait_for_result()


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
    while not rospy.is_shutdown():
        if (py_trees.blackboard.Blackboard().get(BB_VAR_NUM_OF_RESCUED_VICTIMS)
                == num_of_victims_to_rescue):
            rospy.loginfo("Mission completed.")
            break
        """
        When a behaviour tree ticks, it traverses the behaviours (starting at the root of the tree), ticking each behaviour, catching its result and then using that result to make decisions on the direction the tree traversal will take. This is the decision part of the tree. Once the traversal ends back at the root, the tick is over.
        Any blocking work should be happening somewhere else with a behaviour simply in charge of starting/monitoring and catching the result of that work.
        """
        behavior_tree.tick()
        if py_trees.blackboard.Blackboard().get(BB_VAR_RETURNED_HOME):
            rospy.loginfo("Drone returned home.")
            break
        bt_tip = behavior_tree.tip()
        if (
            bt_tip
            # and bt_tip.name == LEAF_CHECK_VICTIM_FOUND_NAME
            and bt_tip.status == py_trees.common.Status.FAILURE
        ):
            break
        rate.sleep()
    bt_tip = behavior_tree.tip()
    if bt_tip and bt_tip.status == py_trees.common.Status.FAILURE:
        lead_drone_into_safe_state()

# Drone world position in centimetres -> Drone position in grid
def world_to_grid(world_x, world_y):
    grid_x = int(world_x / defaults.map_resolution)
    grid_y = int(world_y / defaults.map_resolution)
    return grid_y, grid_x


def grid_to_world(x, y):
    x_world = (x + 0.5) * defaults.map_resolution
    y_world = (y + 0.5) * defaults.map_resolution
    return y_world, x_world


def flat_to_2d(flat_array, width):
    if len(flat_array) % width != 0:
        raise ValueError(
            "The length of the flat array is not evenly divisible by the width"
        )
    return numpy.reshape(flat_array, (width, width))


def parse_waypoints():
    waypoints_str = rospy.get_param("~waypoints_arr")
    waypoints = json.loads(str(waypoints_str))
    # Convert the waypoints to a list of Points
    return deque(
        [Point(waypoints[i], waypoints[i + 1], 0) for i in range(0, len(waypoints), 2)]
    )


if __name__ == "__main__":
    # pathfinding test
    # occupancy_grid = [
    #         100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
    #         100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
    #         100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
    #         100, 100, 100, 100, 100, 100, 100, 1,   100, 100,
    #         100, 100, 100, 100, 100, 100, 100,   0, 100, 100,
    #         100, 100, 100, 100, 100, 100, 100,   0, 100, 100,
    #         100, 100, 100, 100, 100, 100, 100,   0, 100, 100,
    #         100, 100, 0,     0,   0,   0,   0,   0, 100, 100,
    #         100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
    #         100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
    #     ]
    # print(flat_to_2d(occupancy_grid, 10))
    # start = (7, 2)  # Starting cell
    # goal = (3, 7)  # Target cell
    # path = pathfinding.a_star(flat_to_2d(occupancy_grid, 10), start, goal)
    # print(path)
    try:
        # register the node with roscore, allowing it to communicate with other nodes
        rospy.init_node("planner")
        if rospy.get_param("~interactive_waypoints"):
            waypoints = parse_waypoints()
            blackboard = py_trees.blackboard.Blackboard()
            blackboard.waypoints = waypoints
        # for testing purpose
        # py_trees.logging.level = py_trees.logging.Level.DEBUG
        tree = setup_bt()
        py_trees.display.render_dot_tree(tree.root, name="planner_tree")
        run_bt(tree)
    except rospy.ROSInterruptException:
        pass
