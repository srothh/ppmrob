#!/usr/bin/env python3

from collections import deque
import functools
import json

import numpy

import actionlib
import py_trees
import py_trees_ros
import rospy

from geometry_msgs.msg import Point, PoseStamped, PolygonStamped
from nav_msgs.msg import OccupancyGrid
import pathfinding
import common.config.defaults as defaults
import control.msg

# Constants
DRONE_MOVEMENT_INCREMENT = 30

# Blackboard variable names
BB_VAR_RETURNED_HOME = "returned_home"
BB_VAR_VICTIM_LINES = "victim_found"
BB_VAR_WAYPOINT = "waypoint"
BB_VAR_NUM_OF_RESCUED_VICTIMS = "num_of_rescued_victims"
BB_VAR_VICTIM_JUST_SAVED = "victim_just_saved"

BB_VAR_WAYPOINTS = "waypoints"
BB_VAR_MAP_WIDTH = "map_width"
BB_VAR_MAP_DATA = "map_data"
BB_VAR_MAP_ORIGIN = "map_origin"
BB_VAR_WORLD_POS = "world_pos"


class DynamicPlan(py_trees.behaviour.Behaviour):
    """
    Behaviour to dynamically plan a path.
    """

    def __init__(self, name):
        super(DynamicPlan, self).__init__(name)

    def update(self):
        path = dynamic_plan()
        py_trees.blackboard.Blackboard().plan = path
        return py_trees.common.Status.FAILURE if len(path) == 0 else py_trees.common.Status.SUCCESS


class UnexploredPlan(py_trees.behaviour.Behaviour):
    """
    Behaviour to plan a path to unexplored areas.
    """

    def __init__(self, name):
        super(UnexploredPlan, self).__init__(name)

    def update(self):
        path = path_to_unexplored()
        py_trees.blackboard.Blackboard().plan = path
        return py_trees.common.Status.FAILURE if len(path) == 0 else py_trees.common.Status.SUCCESS


class HomePlan(py_trees.behaviour.Behaviour):
    """
    Behaviour to plan a path home.
    """

    def __init__(self, name):
        super(HomePlan, self).__init__(name)

    def update(self):
        path = path_home()
        py_trees.blackboard.Blackboard().plan = path
        return py_trees.common.Status.FAILURE if len(path) == 0 else py_trees.common.Status.SUCCESS


class Leaf(py_trees.Behaviour):
    """
    Base behaviour class for connecting to a subprocess to initiate a goal and monitor its progress.
    """

    def __init__(self, name="Leaf"):
        super(Leaf, self).__init__(name)
        self.logger.debug(f"  {self.name} [{self.__class__.__name__}::__init__()]")

    def setup(self, timeout=15):
        self.logger.debug(f"  {self.name} [{self.__class__.__name__}::setup()]")
        return True

    def initialise(self):
        self.logger.debug(f"  {self.name} [{self.__class__.__name__}::initialise()]")

    def update(self):
        self.logger.debug(f"  {self.name} [{self.__class__.__name__}::update()]")
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.logger.debug(
            f"  {self.name} [{self.__class__.__name__}::terminate()][{self.status}->{new_status}]"
        )


class ReturnHomeDynamicActionClient(py_trees_ros.actions.ActionClient):
    """
    Action client for returning home dynamically.
    """

    def initialise(self):
        planned_path = py_trees.blackboard.Blackboard().plan
        rospy.loginfo(f"Planned path: {planned_path}")
        if len(planned_path) == 0:
            rospy.loginfo("Path could not be calculated")
        else:
            rospy.loginfo("Returning home...")
        self.action_goal = control.msg.PlanningMoveGoal(target=planned_path)
        super().initialise()


class PlanningMoveDynamicActionClient(py_trees_ros.actions.ActionClient):
    """
    Action client for dynamically moving along a planned path.
    """

    def initialise(self):
        planned_path = py_trees.blackboard.Blackboard().plan
        rospy.loginfo(f"Planned path: {planned_path}")
        if len(planned_path) == 0:
            rospy.loginfo("Path could not be calculated")
        self.action_goal = control.msg.PlanningMoveGoal(target=planned_path)
        super().initialise()


class PlanningMoveInteractiveActionClient(py_trees_ros.actions.ActionClient):
    """
    Action client for moving to a waypoint interactively.
    """

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
        blackboard.set(self.variable_name, IncrementBbVar.initialise.counter, overwrite=True)
        blackboard.set(BB_VAR_VICTIM_JUST_SAVED, True, overwrite=True)
        rospy.loginfo(f"Number of rescued victims: {IncrementBbVar.initialise.counter}")


def add_points(point1, point2):
    """
    Adds two geometry_msgs Point objects.
    """
    result = Point()
    result.x = point1.x + point2.x
    result.y = point1.y + point2.y
    result.z = point1.z + point2.z
    return result


def process_map():
    """
    Processes the map data and updates the blackboard.
    """
    grid = flat_to_2d(
        py_trees.blackboard.Blackboard().get(BB_VAR_MAP_DATA),
        py_trees.blackboard.Blackboard().get(BB_VAR_MAP_WIDTH),
    )
    origin = py_trees.blackboard.Blackboard().get(BB_VAR_MAP_ORIGIN)
    world_pos = add_points(py_trees.blackboard.Blackboard().get(BB_VAR_WORLD_POS), origin)
    grid_pos_x, grid_pos_y = world_to_grid(world_pos.x, world_pos.z)
    map = py_trees.blackboard.Blackboard().get("map")
    if len(map) != len(grid):
        map = grid
    map[grid_pos_x][grid_pos_y] = 50
    for row in range(len(grid)):
        for cell in range(len(grid[row])):
            if grid[row][cell] != 50:
                map[row][cell] = grid[row][cell]
    py_trees.blackboard.Blackboard().set("map", map, overwrite=True)


def dynamic_plan():
    """
    Generates a dynamic path plan based on the current map.
    """
    if py_trees.blackboard.Blackboard().get(BB_VAR_MAP_WIDTH) == 0:
        return []

    process_map()
    grid = py_trees.blackboard.Blackboard().get("map")
    origin = py_trees.blackboard.Blackboard().get(BB_VAR_MAP_ORIGIN)
    world_pos = add_points(py_trees.blackboard.Blackboard().get(BB_VAR_WORLD_POS), origin)
    grid_pos_x, grid_pos_y = world_to_grid(world_pos.x, world_pos.z)

    path = []
    if grid[grid_pos_x][grid_pos_y + 1] == 0:
        path = [Point(world_pos.x + DRONE_MOVEMENT_INCREMENT - origin.x, world_pos.y, world_pos.z - origin.z)]
    elif grid[grid_pos_x + 1][grid_pos_y] == 0:
        path = [Point(world_pos.x - origin.x, world_pos.y, world_pos.z + DRONE_MOVEMENT_INCREMENT - origin.z)]
    elif grid[grid_pos_x][grid_pos_y - 1] == 0:
        path = [Point(world_pos.x - DRONE_MOVEMENT_INCREMENT - origin.x, world_pos.y, world_pos.z - origin.z)]
    elif grid[grid_pos_x - 1][grid_pos_y] == 0:
        path = [Point(world_pos.x - origin.x, world_pos.y, world_pos.z - DRONE_MOVEMENT_INCREMENT - origin.z)]
    return path


def path_to_unexplored():
    """
    Generates a path to an unexplored area.
    """
    if py_trees.blackboard.Blackboard().get(BB_VAR_MAP_WIDTH) == 0:
        return []

    grid = py_trees.blackboard.Blackboard().get("map")
    for row in range(len(grid)):
        for cell in range(len(grid[row])):
            if grid[row][cell] == 0:
                path = path_to_pos(row, cell)
                if len(path) != 0:
                    return path
    return []


def path_home():
    """
    Generates a path to the home position.
    """
    origin = py_trees.blackboard.Blackboard().get(BB_VAR_MAP_ORIGIN)
    grid_pos_x, grid_pos_y = world_to_grid(origin.x, origin.z)
    return path_to_pos(grid_pos_x, grid_pos_y)


def path_to_pos(x, y):
    """
    Generates a path to a specific grid position.
    """
    if py_trees.blackboard.Blackboard().get(BB_VAR_MAP_WIDTH) == 0:
        return []

    grid = py_trees.blackboard.Blackboard().get("map")
    origin = py_trees.blackboard.Blackboard().get(BB_VAR_MAP_ORIGIN)
    world_pos = add_points(py_trees.blackboard.Blackboard().get(BB_VAR_WORLD_POS), origin)
    grid_pos_x, grid_pos_y = world_to_grid(world_pos.x, world_pos.z)
    current = (grid_pos_x, grid_pos_y)
    target = (x, y)
    path_indices = pathfinding.a_star(grid, current, target)
    path = []
    for point in path_indices:
        w_pos = grid_to_world(point[0], point[1])
        path.append(
            Point(w_pos[0] - origin.x, py_trees.blackboard.Blackboard().get(BB_VAR_WORLD_POS).y, w_pos[1] - origin.z))
    return path


def create_root_interactive():
    """
    Creates the behaviour tree for interactive waypoints mode and returns the root node.
    """
    rospy.loginfo("Interactive waypoints mode")
    root = py_trees.composites.Parallel("Mission")
    topics2bb = py_trees.composites.Sequence("Topics2BB")
    victim_found2bb = py_trees_ros.subscribers.ToBlackboard(
        name="VictimFound2BB",
        topic_name=defaults.CV.VICTIM_LINES_TOPIC_NAME,
        topic_type=PolygonStamped,
        blackboard_variables={BB_VAR_VICTIM_LINES: "polygon.points"},
        initialise_variables={BB_VAR_VICTIM_LINES: []},
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
        action_goal=control.msg.PlanningCommandGoal(command=defaults.TelloCommands.LAND),
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
        action_goal=control.msg.PlanningCommandGoal(command=defaults.TelloCommands.TAKEOFF),
        action_namespace=defaults.Control.COMMAND_ACTION_NAMESPACE,
        override_feedback_message_on_running="Taking off...",
    )
    search_subtree = py_trees.composites.Sequence(name="Search victim")
    search_subtree_condition = py_trees.decorators.Condition(
        child=search_subtree, status=py_trees.common.Status.FAILURE
    )
    victim_found_or_just_saved = py_trees.composites.Selector("Victim found or just saved")
    skip_victim_found = py_trees.composites.Sequence("Skip victim found")
    victim_just_saved = py_trees.blackboard.CheckBlackboardVariable(
        name="Victim just saved?",
        variable_name=BB_VAR_VICTIM_JUST_SAVED,
        expected_value=True,
    )
    look_for_next_victim = py_trees.blackboard.SetBlackboardVariable(
        name="Look for next victim",
        variable_name=BB_VAR_VICTIM_JUST_SAVED,
        variable_value=False,
    )
    no_victim_found = py_trees.blackboard.CheckBlackboardVariable(
        name="No victim found?",
        variable_name=BB_VAR_VICTIM_LINES,
        expected_value=[],
    )
    move_to_next_position = PlanningMoveInteractiveActionClient(
        name="Move to next position",
        action_spec=control.msg.PlanningMoveAction,
        action_namespace=defaults.Control.MOVE_ACTION_NAMESPACE,
        override_feedback_message_on_running="Moving to next position...",
    )
    is_waypoints_empty = py_trees.meta.inverter(py_trees.blackboard.CheckBlackboardVariable)(
        name="Any waypoints not yet flown?",
        variable_name=BB_VAR_WAYPOINTS,
        expected_value=deque([]),
    )
    rescue_subtree = py_trees.composites.Sequence(name="Rescue victim")
    no_victim_actually_found = py_trees.blackboard.CheckBlackboardVariable(
        name="No victim actually found?",
        variable_name=BB_VAR_VICTIM_LINES,
        expected_value=[],
    )
    no_victim_actually_found_inverter = py_trees.decorators.Inverter(
        child=no_victim_actually_found
    )
    land_where_victim_found = py_trees_ros.actions.ActionClient(
        name="Land where victim found",
        action_spec=control.msg.PlanningCommandAction,
        action_goal=control.msg.PlanningCommandGoal(command=defaults.TelloCommands.LAND),
        action_namespace=defaults.Control.COMMAND_ACTION_NAMESPACE,
        override_feedback_message_on_running="Landing where victim found...",
    )
    victim_rescued = IncrementBbVar("Rescued victim", BB_VAR_NUM_OF_RESCUED_VICTIMS)

    root.add_children([topics2bb, priorities])
    topics2bb.add_children([victim_found2bb, battery2bb])
    priorities.add_children([battery_check, search_and_rescue])
    battery_check.add_children([is_battery_low, return_home])
    return_home.add_children([fly_home, land_home, terminate])
    search_and_rescue.add_children([takeoff, search_subtree_condition, rescue_subtree])
    search_subtree.add_children([victim_found_or_just_saved, move_to_next_position, is_waypoints_empty])
    victim_found_or_just_saved.add_children([skip_victim_found, no_victim_found])
    skip_victim_found.add_children([victim_just_saved, look_for_next_victim])
    rescue_subtree.add_children([no_victim_actually_found_inverter, land_where_victim_found, victim_rescued])

    return root


def create_root():
    """
    Creates the behaviour tree and returns the root node.
    The tree is built as if executing a tree traversal algorithm: from root to bottom left to bottom right nodes.
    """
    rospy.loginfo("Non-interactive waypoints mode")
    root = py_trees.composites.Parallel("Mission")
    topics2bb = py_trees.composites.Sequence("Topics2BB")
    victim_found2bb = py_trees_ros.subscribers.ToBlackboard(
        name="VictimFound2BB",
        topic_name=defaults.CV.VICTIM_LINES_TOPIC_NAME,
        topic_type=PolygonStamped,
        blackboard_variables={BB_VAR_VICTIM_LINES: "polygon.points"},
        initialise_variables={BB_VAR_VICTIM_LINES: []},
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
        blackboard_variables={
            BB_VAR_MAP_WIDTH: "info.width",
            BB_VAR_MAP_DATA: "data",
            BB_VAR_MAP_ORIGIN: "info.origin.position",
        },
        initialise_variables={
            BB_VAR_MAP_WIDTH: 0,
            BB_VAR_MAP_DATA: [],
            BB_VAR_MAP_ORIGIN: Point(0, 0, 0),
        },
    )
    world_pos2bb = py_trees_ros.subscribers.ToBlackboard(
        name="WorldPos2BB",
        topic_name=defaults.Odometry.WORLD_POSITION_TOPIC_NAME,
        topic_type=PoseStamped,
        blackboard_variables={BB_VAR_WORLD_POS: "pose.position"},
        initialise_variables={BB_VAR_WORLD_POS: Point(0, 0, 0)},
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
        action_goal=control.msg.PlanningCommandGoal(command=defaults.TelloCommands.LAND),
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
        action_goal=control.msg.PlanningCommandGoal(command=defaults.TelloCommands.TAKEOFF),
        action_namespace=defaults.Control.COMMAND_ACTION_NAMESPACE,
        override_feedback_message_on_running="Taking off...",
    )
    search_subtree = py_trees.composites.Sequence(name="Search victim")
    search_subtree_condition = py_trees.decorators.Condition(
        child=search_subtree, status=py_trees.common.Status.FAILURE
    )
    victim_found_or_just_saved = py_trees.composites.Selector("Victim found or just saved")
    skip_victim_found = py_trees.composites.Sequence("Skip victim found")
    victim_just_saved = py_trees.blackboard.CheckBlackboardVariable(
        name="Victim just saved?",
        variable_name=BB_VAR_VICTIM_JUST_SAVED,
        expected_value=True,
    )
    look_for_next_victim = py_trees.blackboard.SetBlackboardVariable(
        name="Look for next victim",
        variable_name=BB_VAR_VICTIM_JUST_SAVED,
        variable_value=False,
    )
    no_victim_found = py_trees.blackboard.CheckBlackboardVariable(
        name="No victim found?",
        variable_name=BB_VAR_VICTIM_LINES,
        expected_value=[],
    )
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
    no_victim_actually_found = py_trees.blackboard.CheckBlackboardVariable(
        name="No victim actually found?",
        variable_name=BB_VAR_VICTIM_LINES,
        expected_value=[],
    )
    no_victim_actually_found_inverter = py_trees.decorators.Inverter(
        child=no_victim_actually_found
    )
    land_where_victim_found = py_trees_ros.actions.ActionClient(
        name="Land where victim found",
        action_spec=control.msg.PlanningCommandAction,
        action_goal=control.msg.PlanningCommandGoal(command=defaults.TelloCommands.LAND),
        action_namespace=defaults.Control.COMMAND_ACTION_NAMESPACE,
        override_feedback_message_on_running="Landing where victim found...",
    )
    victim_rescued = IncrementBbVar("Rescued victim", BB_VAR_NUM_OF_RESCUED_VICTIMS)

    root.add_children([topics2bb, priorities])
    topics2bb.add_children([map2bb, world_pos2bb, victim_found2bb, battery2bb])
    priorities.add_children([battery_check, search_and_rescue])
    battery_check.add_children([is_battery_low, return_home])
    return_home.add_children([plan_home, fly_home, land_home, terminate])
    search_and_rescue.add_children([takeoff, search_subtree_condition, rescue_subtree])
    search_subtree.add_children([victim_found_or_just_saved, path_planning, move_to_next_position])
    victim_found_or_just_saved.add_children([skip_victim_found, no_victim_found])
    skip_victim_found.add_children([victim_just_saved, look_for_next_victim])
    path_planning.add_children([dynamic, unexplored, plan_home])
    rescue_subtree.add_children([no_victim_actually_found_inverter, land_where_victim_found, victim_rescued])

    return root


def post_tick_handler(snapshot_visitor, tree):
    """
    Post-tick handler for printing the tree in the console.
    """
    py_trees.display.print_ascii_tree(tree.root, show_status=True)


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
    snapshot_visitor = py_trees.visitors.SnapshotVisitor()
    tree.add_post_tick_handler(functools.partial(post_tick_handler, snapshot_visitor))
    tree.visitors.append(py_trees.visitors.DebugVisitor())
    tree.visitors.append(snapshot_visitor)
    if not tree.setup(timeout=timeout):
        rospy.loginfo("Setup of BT failed")
    rospy.loginfo("Behaviour tree created.")
    return tree


def lead_drone_into_safe_state():
    """
    If the drone is in the air, it will be stopped and landed.
    """
    ac_stop_cmd = actionlib.SimpleActionClient(
        defaults.Control.COMMAND_ACTION_NAMESPACE, control.msg.PlanningCommandAction
    )
    ac_stop_cmd.wait_for_server()
    ac_stop_cmd.send_goal(control.msg.PlanningCommandGoal(command=defaults.TelloCommands.STOP))
    ac_stop_cmd.wait_for_result()
    ac_land_cmd = actionlib.SimpleActionClient(
        defaults.Control.COMMAND_ACTION_NAMESPACE, control.msg.PlanningCommandAction
    )
    ac_land_cmd.wait_for_server()
    ac_land_cmd.send_goal(control.msg.PlanningCommandGoal(command=defaults.TelloCommands.LAND))
    ac_land_cmd.wait_for_result()
    rospy.loginfo("Drone in safe state.")


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
    num_of_victims_to_rescue = rospy.get_param("~num_of_victims_to_rescue")
    rospy.loginfo(f"Number of victims to rescue: {num_of_victims_to_rescue}")
    while not rospy.is_shutdown():
        if py_trees.blackboard.Blackboard().get(BB_VAR_NUM_OF_RESCUED_VICTIMS) == num_of_victims_to_rescue:
            rospy.loginfo("Mission completed.")
            break
        behavior_tree.tick()
        if py_trees.blackboard.Blackboard().get(BB_VAR_RETURNED_HOME):
            rospy.loginfo("Drone returned home.")
            break
        bt_tip = behavior_tree.tip()
        if bt_tip and bt_tip.status == py_trees.common.Status.FAILURE:
            break
        rate.sleep()
    bt_tip = behavior_tree.tip()
    if bt_tip and bt_tip.status == py_trees.common.Status.FAILURE:
        rospy.loginfo("Behavior tree failed.")
        lead_drone_into_safe_state()


def world_to_grid(world_x, world_y):
    """
    Converts world coordinates to grid coordinates.
    """
    grid_x = int(world_x / defaults.map_resolution)
    grid_y = int(world_y / defaults.map_resolution)
    return grid_y, grid_x


def grid_to_world(x, y):
    """
    Converts grid coordinates to world coordinates.
    """
    x_world = (x + 0.5) * defaults.map_resolution
    y_world = (y + 0.5) * defaults.map_resolution
    return y_world, x_world


def flat_to_2d(flat_array, width):
    """
    Converts a flat array to a 2D array.
    """
    if len(flat_array) % width != 0:
        raise ValueError("The length of the flat array is not evenly divisible by the width")
    return numpy.reshape(flat_array, (width, width))


def parse_waypoints():
    """
    Parses waypoints from a ROS parameter.
    """
    waypoints_str = rospy.get_param("~waypoints_arr")
    waypoints = json.loads(str(waypoints_str))
    return deque([Point(waypoints[i], waypoints[i + 1], 0) for i in range(0, len(waypoints), 2)])


if __name__ == "__main__":
    try:
        # register the node with roscore, allowing it to communicate with other nodes
        rospy.init_node("planner")
        if rospy.get_param("~interactive_waypoints"):
            waypoints = parse_waypoints()
            rospy.loginfo(f"Parsed waypoints: {waypoints}")
            blackboard = py_trees.blackboard.Blackboard()
            blackboard.waypoints = waypoints
        # for testing purpose
        # py_trees.logging.level = py_trees.logging.Level.DEBUG

        py_trees.blackboard.Blackboard().set("map", [])

        tree = setup_bt()
        # py_trees.display.render_dot_tree(tree.root, name="planner_tree")
        run_bt(tree)
    except rospy.ROSInterruptException:
        pass
