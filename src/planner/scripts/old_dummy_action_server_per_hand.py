#!/usr/bin/env python3

import rospy
import actionlib  # import the actionlib library used for implementing simple actions
import control.msg  # The action specification generates several messages for sending goals, receiving feedback, etc... This line imports the generated messages.
import common.config.defaults as defaults


class CommandAction(object):
    # create messages that are used to publish feedback/result
    _feedback = control.msg.PlanningCommandFeedback()
    _result = control.msg.PlanningCommandResult()

    def __init__(self, name):
        self._action_name = name
        """
        Here, the SimpleActionServer is created, we pass it a name (used as a namespace), an action type, and optionally an execute callback. Since we've specified an execute callback in this example, a thread will be spun for us which allows us to take long running actions in a callback received when a new goal comes in.
        """
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            control.msg.PlanningCommandAction,
            execute_cb=self.execute_cb,  # that we'll run everytime a new goal is received
            auto_start=False,  # always, unless you know what you're doing
        )
        self._as.start()

    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(0.25)
        success = True
        # append the seeds for the fibonacci sequence - in tut
        self._feedback.progress = []
        self._feedback.progress.append(True)
        # publish info to the console for the user
        rospy.loginfo(
            "%s: Executing command %s with progress %s"
            % (self._action_name, goal.command, self._feedback.progress[0])
        )
        # The internals of the action are created above.
        # start executing the action
        for i in range(1, 5):  # in tut: (1, goal.order)
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                """
                An important component of an action server is the ability to allow an action client to request that the goal under execution be canceled. When a client requests that the current goal be preempted, the action server should cancel the goal, perform any necessary cleanup, and call the `set_preempted` function, which signals that the action has been preempted by user request. Here, we'll check if we've been preempted every second. We could, alternatively, receive a callback when a preempt request is received.
                """
                rospy.loginfo("%s: Preempted" % self._action_name)
                self._as.set_preempted()
                success = False
                break
            self._feedback.progress.append(True)
            # publish the feedback on the feedback channel provided by the action server
            self._as.publish_feedback(self._feedback)
            # this step is not necessary, the progress is created at 1 Hz for demonstration purposes
            r.sleep()
        if success:
            self._result.command_executed = True
            rospy.loginfo("%s: Succeeded" % self._action_name)
            # the action server notifies the action client that the goal is complete
            self._as.set_succeeded(self._result)


if __name__ == "__main__":
    rospy.init_node(defaults.Control.COMMAND_ACTION_NAMESPACE)
    server = CommandAction(rospy.get_name())
    rospy.spin()
