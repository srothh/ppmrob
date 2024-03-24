#!/usr/bin/env python3
import functools
import actionlib
import py_trees
import py_trees_ros
import rospy
# import control.msg
import control.msg
# import grid_based_sweep
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
import pathfinding
import common.config.defaults as defaults  # TODO add to dockerfile as per issue!

BB_VAR_RETURNED_HOME = "returned_home"
BB_VAR_HOME_COORDINATES = "home_coordinates"
BB_VAR_VICTIM_FOUND = "victim_found"
BB_VAR_WAYPOINTS = "waypoints"


class DynamicPlan(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(DynamicPlan, self).__init__(name)

    def update(self):
        path = dynamic_plan()
        py_trees.blackboard.Blackboard().plan = path
        return py_trees.common.Status.FAILURE if len(path) == 0 else py_trees.common.Status.SUCCESS


class UnexploredPlan(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(UnexploredPlan, self).__init__(name)

    def update(self):
        path = path_to_unexplored()
        py_trees.blackboard.Blackboard().plan = path
        return py_trees.common.Status.FAILURE if len(path) == 0 else py_trees.common.Status.SUCCESS


class HomePlan(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(HomePlan, self).__init__(name)

    def update(self):
        path = path_home()
        py_trees.blackboard.Blackboard().plan = path
        return py_trees.common.Status.FAILURE if len(path) == 0 else py_trees.common.Status.SUCCESS


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
        planned_path = py_trees.blackboard.Blackboard().get(BB_VAR_WAYPOINTS)
        # TODO check that this works as intended!
        self.action_goal = control.msg.PlanningMoveGoal(target=planned_path)
        super().initialise()


class PlanningMoveDynamicActionClient(py_trees_ros.actions.ActionClient):
    def initialise(self):
        planned_path = py_trees.blackboard.Blackboard().get(BB_VAR_WAYPOINTS)
        # TODO check that this works as intended!
        self.action_goal = control.msg.PlanningMoveGoal(target=planned_path)
        super().initialise()


def dynamic_plan():
    grid = py_trees.blackboard.Blackboard().map_data
    grid_pos_y, grid_pos_x = world_to_grid(py_trees.blackboard.Blackboard().world_pos.x,
                                           py_trees.blackboard.Blackboard().world_pos.y)
    path = []
    if grid[grid_pos_x][grid_pos_y + 1] == 0 or grid[grid_pos_x][grid_pos_y + 1] == 1:
        path = [(py_trees.blackboard.Blackboard().world_pos.x, py_trees.blackboard.Blackboard().world_pos.y + 30)]
    elif grid[grid_pos_x + 1][grid_pos_y] == 0 or grid[grid_pos_x + 1][grid_pos_y] == 1:
        path = [(py_trees.blackboard.Blackboard().world_pos.x + 30, py_trees.blackboard.Blackboard().world_pos.y)]
    elif grid[grid_pos_x][grid_pos_y - 1] == 0 or grid[grid_pos_x][grid_pos_y - 1] == 0:
        path = [(py_trees.blackboard.Blackboard().world_pos.x, py_trees.blackboard.Blackboard().world_pos.y - 30)]
    elif grid[grid_pos_x - 1][grid_pos_y] == 0 or grid[grid_pos_x - 1][grid_pos_y] == 0:
        path = [(py_trees.blackboard.Blackboard().world_pos.x - 30, py_trees.blackboard.Blackboard().world_pos.y)]
    return path


def path_to_unexplored():
    path = []
    grid = py_trees.blackboard.Blackboard().map_data
    for row in range(len(grid)):
        for cell in range(len(row)):
            if grid[row][cell] == 1:
                path = path_to_pos(row, cell)
    return path


def path_home():
    return path_to_pos(0, 0)


def path_to_pos(row, cell):
    grid = py_trees.blackboard.Blackboard().map_data
    grid_pos_y, grid_pos_x = world_to_grid(py_trees.blackboard.Blackboard().world_pos.x,
                                           py_trees.blackboard.Blackboard().world_pos.y)
    current = Point(grid_pos_x, grid_pos_y, 0)
    target = Point(row, cell, 0)
    return pathfinding.a_star(grid, (current.x, current.y), (target.x, target.y))


def create_root():
    """
    Creates the behaviour tree and returns the root node.
    The tree is built as if executing a tree traversal algorithm: from root to bottom left to bottom right nodes.
    """
    # nodes
    root = py_trees.composites.Parallel("Mission")
    topics2bb = py_trees.composites.Sequence("Topics2BB")
    # TODO adjust our architecture to use this! - just ME: see tut for on which topic to publish to and which msg (only percentage)
    battery2bb = py_trees_ros.battery.ToBlackboard(
        name="Battery2BB",
        topic_name=defaults.drone_battery_sensor_publish_topic_name,
        threshold=defaults.Drone.BATTERY_THRESHOLD,
    )
    # preferred way of getting data from a topic according to docs
    home_coords2bb = py_trees_ros.subscribers.ToBlackboard(
        name="HomeCoords2BB",
        topic_name=defaults.Odometry.HOME_COORDS_TOPIC_NAME,
        topic_type=Point,
        blackboard_variables={BB_VAR_HOME_COORDINATES: None},
        initialise_variables={BB_VAR_HOME_COORDINATES: Point()},
    )
    victim_found2bb = py_trees_ros.subscribers.ToBlackboard(
        name="VictimFound2BB",
        topic_name=defaults.Mapping.VICTIM_FOUND_TOPIC_NAME,
        topic_type=Bool,
        # get rid of the annoying sub-data field
        blackboard_variables={BB_VAR_VICTIM_FOUND: "data"},
        initialise_variables={BB_VAR_VICTIM_FOUND: False},
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
    )
    terminate = py_trees.blackboard.SetBlackboardVariable(
        name="Terminate", variable_name=BB_VAR_RETURNED_HOME, variable_value=True
    )
    search_and_rescue = py_trees.composites.Sequence(name="Search & rescue victim")
    search_subtree = py_trees.composites.Sequence(name="Search victim")
    search_subtree_condition = py_trees.decorators.Condition(
        child=search_subtree, status=py_trees.common.Status.FAILURE
    )
    takeoff = py_trees_ros.actions.ActionClient(
        name="Takeoff",
        action_spec=control.msg.PlanningCommandAction,
        action_goal=control.msg.PlanningCommandGoal(
            command=defaults.TelloCommands.TAKEOFF
        ),
        action_namespace=defaults.Control.COMMAND_ACTION_NAMESPACE,
    )
    is_victim_found = py_trees.blackboard.CheckBlackboardVariable(
        name="Victim found?",
        variable_name=BB_VAR_VICTIM_FOUND,
        expected_value=True,
    )
    is_victim_found_inverter = py_trees.decorators.Inverter(child=is_victim_found)
    path_planning = py_trees.composites.Selector("Path planning")
    dynamic_plan = DynamicPlan("Dynamic planning")
    unexplored = UnexploredPlan("Plan path to unexplored cell")
    path_planning.add_children([dynamic_plan, unexplored, plan_home])
    # FIXME following needs output of path planning! -> make dynamic action client
    move_to_next_position = PlanningMoveDynamicActionClient(
        name="Move to next position",
        action_spec=control.msg.PlanningMoveAction,
        action_namespace=defaults.Control.MOVE_ACTION_NAMESPACE,
    )
    rescue_subtree = py_trees.composites.Sequence(name="Rescue victim")
    stop_above_victim = py_trees_ros.actions.ActionClient(
        name="Stop above victim",
        action_spec=control.msg.PlanningCommandAction,
        action_goal=control.msg.PlanningCommandGoal(
            command=defaults.TelloCommands.STOP
        ),
        action_namespace=defaults.Control.COMMAND_ACTION_NAMESPACE,
    )
    land_where_victim_found = py_trees_ros.actions.ActionClient(
        name="Land where victim found",
        action_spec=control.msg.PlanningCommandAction,
        action_goal=control.msg.PlanningCommandGoal(
            command=defaults.TelloCommands.LAND
        ),
        action_namespace=defaults.Control.COMMAND_ACTION_NAMESPACE,
    )
    # tree
    root.add_children([topics2bb, priorities])
    topics2bb.add_children([battery2bb, home_coords2bb, victim_found2bb])
    priorities.add_children([battery_check, search_and_rescue])
    battery_check.add_children([is_battery_low, return_home])
    return_home.add_children([plan_home, fly_home, land_home, terminate])
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


# Map callback function
def map_callback(msg):
    py_trees.blackboard.Blackboard().map_width = msg.info.width
    py_trees.blackboard.Blackboard().map_data = flat_to_2d(msg.data, msg.info.width)


# Position callback function
def position_callback(msg):
    py_trees.blackboard.Blackboard().world_pos = msg


# Drone world position in centimetres -> Drone position in grid
def world_to_grid(world_x, world_y):
    grid_x = int(world_x / defaults.map_resolution)
    grid_y = int(world_y / defaults.map_resolution)
    return grid_x, grid_y


def grid_to_world(row, col):
    x_world = col * defaults.map_resolution / 2
    y_world = row * defaults.map_resolution / 2
    return x_world, y_world


def flat_to_2d(flat_array, width):
    if len(flat_array) % width != 0:
        raise ValueError("The length of the flat array is not evenly divisible by the width")
    return [flat_array[i:i + width] for i in range(0, len(flat_array), width)]


if __name__ == "__main__":
    # pathfinding test
    # occupancy_grid = [
    #     9,9,9,9,9,9,9,9,9,9,
    #     9,0,0,0,0,0,0,0,0,9,
    #     9,0,0,0,0,0,0,0,0,9,
    #     9,9,9,9,9,9,0,0,0,9,
    #     9,9,9,9,9,9,0,0,0,9,
    #     9,9,9,9,9,9,0,0,0,9,
    #     9,0,0,0,0,0,0,0,0,9,
    #     9,0,2,2,2,2,2,2,2,9,
    #     9,2,2,1,1,1,1,1,1,1,
    #     1,1,1,1,1,1,1,1,1,1
    # ]
    # start = (7, 1)  # Starting cell
    # goal = (1, 1)  # Target cell
    # path = pathfinding.a_star(flat_to_2d(occupancy_grid, 10), start, goal)
    # print(path)
    try:
        # register the node with roscore, allowing it to communicate with other nodes
        rospy.init_node("planner")

        # Occupancy grid subscriber
        map_subscriber = rospy.Subscriber(defaults.Mapping.OCCUPANCY_GRID_TOPIC_NAME, OccupancyGrid, map_callback)
        # World position subscriber
        position_subscriber = rospy.Subscriber(defaults.Control.WORLD_POSITION_TOPIC_NAME, Point, position_callback)

        # for testing purpose
        py_trees.logging.level = py_trees.logging.Level.DEBUG
        tree = setup_bt()
        py_trees.display.render_dot_tree(tree.root, name="planner_tree")
        run_bt(tree)
    except rospy.ROSInterruptException:
        pass
