#!/usr/bin/env python3
import functools

import actionlib
import py_trees
import py_trees_ros
import rospy
import planner.msg

import grid_based_sweep
from std_msgs.msg import String


class TelloCommands:
    TAKEOFF = "takeoff"
    LAND = "land"
    STOP = "stop"

class VictimFound(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(VictimFound, self).__init__(name)
        self.victim_found_subscriber = rospy.Subscriber("/mapping/victim_found", String, self.victim_found_callback)
        self.victim_found = False

    def victim_found_callback(self, msg):
        self.victim_found = msg.data == "True"

    def initialise(self):
        self.victim_found = False

    def update(self):
        if self.victim_found:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        pass

class CreatePlan(py_trees.behaviour.Behaviour):
    def __init__(self, name, action_server_name):
        super(CreatePlan, self).__init__(name)
        self.map_subscriber = rospy.Subscriber("/mapping/map", String, self.map_callback)
        self.map_data = None
        self.action_client = actionlib.SimpleActionClient(action_server_name, planner.msg.PlanAction)
        self.action_client.wait_for_server()

    def map_callback(self, msg):
        self.map_data = msg

    def initialise(self):
        pass

    def update(self):
        if self.map_data is None:
            rospy.loginfo("Waiting for map data...")
            return py_trees.common.Status.RUNNING

        plan = grid_based_sweep.generate_plan()
        goal = planner.msg.PlanGoal()
        goal.x = plan[0]
        goal.y = plan[1]

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
            - never blocks, at most it just monitors the progress and holds up any decision making required by a tree that is ticking the behaviour by setting its status to RUNNING.
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


class MyDynamicActionClient(py_trees_ros.actions.ActionClient):
    def initialise(self):
        home_coords = py_trees.blackboard.Blackboard().get("home_coordinates")
        self.action_goal = planner.msg.MoveGoal(
            x=home_coords.x,  # FIXME
            y=home_coords.y,  # FIXME
        )
        super().initialise()

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.SUCCESS:
            py_trees.blackboard.Blackboard().set("returned_home ", True)
        super().terminate(new_status)


def create_root():
    # TODO build the final tree as if executing tree traversal algorithm: from root to bottom left to bottom right
    # nodes
    root = py_trees.composites.Parallel("Mission")
    topics2bb = py_trees.composites.Sequence("Topics2BB")
    # TODO change the topic name and adjust our architecture to use this!
    # TODO use the constant from common folder for threshold!
    battery2bb = py_trees_ros.battery.ToBlackboard(
        name="Battery2BB", topic_name="/battery/state", threshold=10.0
    )  # see tut for on which topic to publish to and which msg (only percentage)
    # TODO # replace with the actual topic name!
    # home_coords2bb = py_trees_ros.subscribers.ToBlackboard(
    #     name="HomeCoords2BB",
    #     topic_name="/mapping/home_coordinates",
    #     blackboard_variables={"home_coordinates": None},
    # )

    topics2bb.add_children([battery2bb])
    priorities = py_trees.composites.Selector("Priorities")


    battery_check = py_trees.composites.Sequence("BatteryCheck")
    low_battery = py_trees.behaviours.Success("Low battery?")
    #battery_inv = py_trees.decorators.Inverter(child=low_battery)
    return_home = py_trees.behaviours.Success("Return home")

    battery_check.add_children([low_battery, return_home])

    search_and_rescue = py_trees.composites.Selector("Search and Rescue")

    search = py_trees.composites.Sequence("Search")
    takeoff = py_trees_ros.actions.ActionClient(
        name="Takeoff",
        action_spec=planner.msg.CommandAction,
        action_goal=planner.msg.CommandGoal(command=TelloCommands.TAKEOFF),
        action_namespace="command",
    )
    vic_found = VictimFound("victim found")
    inverter = py_trees.decorators.Inverter(child=vic_found)
    update_plan = CreatePlan("Update plan with new map", "/control/plan")
    move = py_trees.behaviours.Success("Move to next cell")
    search.add_children([takeoff, inverter, update_plan, move])

    rescue = py_trees.composites.Sequence("Rescue")
    stop = py_trees_ros.actions.ActionClient(
        name="Stop above victim",
        action_spec=planner.msg.CommandAction,
        action_goal=planner.msg.CommandGoal(command=TelloCommands.STOP),
        action_namespace="command",
    )
    land = py_trees_ros.actions.ActionClient(
        name="Land drone",
        action_spec=planner.msg.CommandAction,
        action_goal=planner.msg.CommandGoal(command=TelloCommands.LAND),
        action_namespace="command",
    )
    rescue.add_children([stop, land])

    search_and_rescue.add_children([search, rescue])

    priorities.add_children([battery_check, search_and_rescue])

    root.add_children([topics2bb, priorities])




    '''
    battery_check = py_trees.composites.Selector("Battery check")
    battery_check_decorator = py_trees.decorators.SuccessIsFailure(child=battery_check)
    is_battery_ok = py_trees.blackboard.CheckBlackboardVariable(
        name="Is battery low?",
        variable_name="battery_low_warning",
        expected_value=False,
    )
    # FIXME in this case ask mapping for home coordinates and send them per move action to control node!
    return_home = MyDynamicActionClient(
        name="Return home",
        action_spec=planner.msg.MoveAction,
        action_namespace="move",
    )
    search_and_rescue = py_trees.composites.Sequence(name="Search & rescue victim")
    search_subtree = py_trees.composites.Sequence(name="Search victim")
    takeoff = py_trees_ros.actions.ActionClient(
        name="Takeoff",
        action_spec=planner.msg.CommandAction,
        action_goal=planner.msg.CommandGoal(command=TelloCommands.TAKEOFF),
        action_namespace="command",
    )
    # TODO build the rest of the search subtree
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
    # tree
    root.add_children([topics2bb, priorities])
    # topics2bb.add_children([battery2bb, home_coords2bb]) # FIXME!
    topics2bb.add_children([battery2bb])
    priorities.add_children([battery_check_decorator, search_and_rescue])
    battery_check.add_children([is_battery_ok, return_home])
    search_and_rescue.add_children([search_subtree, rescue_subtree])

    move_to_next = py_trees.behaviours.Success("Move to next cell")
    cell_processor = py_trees.composites.Sequence("Process current cell")
    update_plan = py_trees.behaviours.Success("Get current map/generate plan")
    vic_classification = py_trees.behaviours.Success("Victim classification")
    cell_processor.add_children([update_plan, vic_classification])
    exploration_sequence = py_trees.composites.Sequence("Exploration", children=[cell_processor, move_to_next])
    search_subtree.add_children([takeoff, exploration_sequence])  # TODO fill the search subtree

    rescue_subtree.add_children([stop_above_victim, land_where_victim_found])
    '''

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
    # TODO is 15 seconds enough? what setup is needed anyway - the action servers?
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
        behavior_tree.tick()  # TODO add the conditions as pre- and post_tick_handlers!
        """
        When a behaviour tree ticks, it traverses the behaviours (starting at the root of the tree), ticking each behaviour, catching its result and then using that result to make decisions on the direction the tree traversal will take. This is the decision part of the tree. Once the traversal ends back at the root, the tick is over.
        Any blocking work should be happening somewhere else with a behaviour simply in charge of starting/monitoring and catching the result of that work.
        """
        if py_trees.blackboard.Blackboard().get("returned_home"):
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
