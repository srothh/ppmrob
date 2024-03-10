#!/usr/bin/env python3
import actionlib
import actionlib_msgs
import functools
import py_trees
import py_trees_ros
import py_trees.console as console
import rospy
import sys
import grid_based_sweep
#from planner.scripts.planner_node import TelloCommands
import planner.msg



class Explore(py_trees.behaviours.Behaviour):
    def __init__(self, name):
        super(Explore, self).__init__(name)
# class Repeater(py_trees.composites.Composite):
#     def __init__(self, name="Repeater", children=None, executions=1, *args, **kwargs):
#         super(Repeater, self).__init__(name, children, *args, **kwargs)
#         self.current_child = None
#         self.count = 0
#         self.executions = executions
class FailUntil(py_trees.behaviours.Behaviour):
    def __init__(self, name="FailUntil", cap=1, reset=True, *args, **kwargs):
        super(FailUntil, self).__init__(name, *args, **kwargs)
        self.count = 0
        self.cap = cap
        self.number_count_resets = 0
        self.number_updated = 0
        self.reset = reset
    # def terminate(self, new_status):
    #     # reset only if udpate got us into an invalid state
    #     if new_status == py_trees.Status.INVALID and self.reset:
    #         self.count = 0
    #         self.number_count_resets += 1
    def update(self):
        self.number_updated += 1
        self.count += 1
        if self.count < self.cap:
            grid_based_sweep.generate_plan()
            return py_trees.Status.FAILURE
        else:
            return py_trees.Status.SUCCESS
class Action_IdentifyNextCell(py_trees_ros.actions.ActionClient):
    def __init__(self, name="Action_IdentifyNextCell", action_spec=None, action_goal=None, action_namespace="/action",
                 override_feedback_message_on_running="moving"):
        super(Action_IdentifyNextCell, self).__init__(name)
        self.action_client = None
        self.sent_goal = False
        self.action_spec = action_spec
        self.action_goal = action_goal
        self.action_namespace = action_namespace
        self.override_feedback_message_on_running = override_feedback_message_on_running
    def setup(self, timeout):
        """
        Args:
            timeout (:obj:`float`): time to wait (0.0 is blocking forever)
        Returns:
            :obj:`bool`: whether it timed out trying to setup
        """
        self.logger.debug("%s.setup()" % self.__class__.__name__)
        self.action_client = actionlib.SimpleActionClient(
            self.action_namespace,
            self.action_spec
        )
        if not self.action_client.wait_for_server(rospy.Duration(timeout)):
            self.logger.error(
                "{0}.setup() could not connect to the rotate action server at '{1}'".format(self.__class__.__name__,
                                                                                            self.action_namespace))
            self.action_client = None
            return False
        return True
    def initialise(self):
        """
        Reset the internal variables.
        """
        self.logger.debug("{0}.initialise()".format(self.__class__.__name__))
        self.sent_goal = False
    def update(self):
        """
        Check only to see whether the underlying action server has
        succeeded, is running, or has cancelled/aborted for some reason and
        map these to the usual behaviour return states.
        """
        self.logger.debug("{0}.update()".format(self.__class__.__name__))
        if not self.action_client:
            self.feedback_message = "no action client, did you call setup() on your tree?"
            return py_trees.Status.INVALID
        # pity there is no 'is_connected' api like there is for c++
        if not self.sent_goal:
            self.action_client.send_goal(self.action_goal, feedback_message=self.feedback_callback)
            self.sent_goal = True
            self.feedback_message = "sent goal to the action server"
            return py_trees.Status.RUNNING
        self.feedback_message = self.action_client.get_goal_status_text()
        if self.action_client.get_state() in [actionlib_msgs.GoalStatus.ABORTED,
                                              actionlib_msgs.GoalStatus.PREEMPTED]:
            return py_trees.Status.FAILURE
        result = self.action_client.get_result()
        if result:
            return py_trees.Status.SUCCESS
        else:
            self.feedback_message = self.override_feedback_message_on_running
            return py_trees.Status.RUNNING
    def feedback_callback(self, feedback):
        print("received feedback:",feedback)
    def terminate(self, new_status):
        """
        If running and the current goal has not already succeeded, cancel it.
        Args:
            new_status (:class:`~py_trees.common.Status`): the behaviour is transitioning to this new status
        """
        self.logger.debug("%s.terminate(%s)" % (self.__class__.__name__, "%s->%s" % (
        self.status, new_status) if self.status != new_status else "%s" % new_status))
        if self.action_client is not None and self.sent_goal:
            motion_state = self.action_client.get_state()
            if ((motion_state == actionlib_msgs.GoalStatus.PENDING) or (
                    motion_state == actionlib_msgs.GoalStatus.ACTIVE) or
                    (motion_state == actionlib_msgs.GoalStatus.PREEMPTING) or (
                            motion_state == actionlib_msgs.GoalStatus.RECALLING)):
                self.action_client.cancel_goal()
        self.sent_goal = False
class RunTest(py_trees.behaviours.Behaviour):
    def __init__(self, name, c, fail=False):
        super(RunTest, self).__init__(name)
        self.response = py_trees.Status.RUNNING
        self.count = 0
        self.max = c
        self.fail = fail
    def update(self):
        self.count += 1
        if self.count == self.max:
            self.count = 0
            self.response = py_trees.Status.SUCCESS if not self.fail else py_trees.Status.FAILURE
        else:
            self.response = py_trees.Status.RUNNING
        print("count ", self.name, ": ", self.count, " | response: ", self.response)
        return self.response
def create_root():
    """
    Create a basic tree and start a 'Topics2BB' work sequence that
    takes the asynchronicity out of subscription.
    Returns:
        :class:`~py_trees.behaviour.Behaviour`: the root of the tree
    """
    # root = py_trees.composites.Sequence("Test")
    #
    # temp = py_trees.composites.Sequence("Test lvl 2")
    #
    # dumb_run = RunTest("Test", 5)
    # cnt = FailUntil("FailUntil", cap=3)
    #
    # temp.add_children([dumb_run, cnt])
    # root.add_children([temp, py_trees.behaviours.Failure("alwaysfail")])
    # --------------------------------
    # test1 = RunTest("test1", 5, False)
    # test2 = RunTest("test2", 6, True)
    # root.add_children([test1, test2])
    # --------------------------------
    identify_cell = py_trees_ros.actions.ActionClient(
        name="identify_next_cell",
        action_spec=plan_msgs.action.PlanAction,
        action_goal=plan_msgs.msg.CommandGoal(command=TelloCommands.STOP),
        action_namespace="/mapping/cell_discovery")
    # --------------------------------
    root = py_trees.composites.Sequence("Rescuer drone")
    rescue_loop = py_trees.composites.Sequence("Rescue loop")
    rescue_runner = py_trees.decorators.FailureIsRunning(name="RescueRunner", child=rescue_loop)
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
def shutdown(behaviour_tree):
    behaviour_tree.interrupt()
if __name__ == '__main__':
    rospy.init_node("tree")
    root = create_root()
    # battery = battery.Battery()
    behaviour_tree = py_trees_ros.trees.BehaviourTree(root)
    rospy.on_shutdown(functools.partial(shutdown, behaviour_tree))
    if not behaviour_tree.setup(timeout=15):
        console.logerror("failed to setup the tree, aborting.")
        sys.exit(1)
    behaviour_tree.tick_tock(1000)
#
# #!/usr/bin/env python3
#
# import rospy
# import actionlib
# from std_msgs.msg import String
# from plan_msgs.msg import PlanAction
#
# class Planner:
#
#     #class variables
#     _feedback = String
#     _result = String
#
#     def __init__(self):  # Replace with your actual topic name
#         rospy.init_node('planner')
#
#         # Define the message type
#         message_type = String  # Replace with the actual message type you want to subscribe to
#
#         # Create a subscriber with the specified topic and message type, and link it to the callback function
#         rospy.Subscriber('/tello_data', message_type, self.control_callback)
#
#         # # Create an action client
#         # client = actionlib.SimpleActionClient('example_action', ExampleAction)
#         # client.wait_for_server()
#
#         # Create an action server
#         server = actionlib.SimpleActionServer('execute_plan_action', plan_msgs.msg.PlanAction, self.execute_plan, False)
#         server.start()
#
#     def control_callback(self, data):
#         # Callback function to process the received message
#         rospy.loginfo("Received message: %s", data.data)
#
#     def execute_plan(self, goal):
#         # Action server execution logic
#         rospy.loginfo(f"Received goal: {goal.input}")
#
#         # Perform some action (in this case, echoing the input)
#         result = ExampleResult()
#         result.output = goal.input
#
#         # Send the result back to the client
#         self.server.set_succeeded(result)
#
#     def send_goal(self, input_text):
#         # Create an action goal
#         goal = ExampleGoal()
#         goal.input = input_text
#
#         # Send the goal to the action server
#         self.client.send_goal(goal)
#
#         # Wait for the result
#         self.client.wait_for_result()
#
#         # Print the result
#         result = self.client.get_result()
#         rospy.loginfo(f"Received result: {result.output}")
#
#     def run(self):
#         # Keep the program alive to listen for messages
#         rospy.spin()
#
# if __name__ == '__main__':
#     planner = Planner()
#     planner.run()
#
#
