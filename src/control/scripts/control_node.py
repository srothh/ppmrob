#!/usr/bin/env python3
import math
import time
import actionlib
import rospy
from action import DroneMoveCommand
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from commands.takeoff_land_handler import TakeOffAndLandHandler
from geometry_msgs.msg import Vector3
from control.msg import (
    MoveResult,
    MoveFeedback,
    PlanningMoveAction,
    PlanningMoveResult,
    PlanningMoveFeedback,
    PlanningCommandAction,
    PlanningCommandResult
)

from common.config.defaults import Control, TelloCommands


class DroneControl:
    def __init__(self):
        # Class Variables from different nodes
        self.prev = Vector3(0.0, 0.0, 0.0)
        self.takeOff = False
        self.land = False
        self.prev_x = None
        self.prev_y = None
        self.target_x = None
        self.target_y = None
        self.prev_course = 0.0
        self.drone_data = None
        self.planning_data = None
        self.mapping_data = None
        rospy.init_node("control_node", anonymous=True)

        rospy.Subscriber("drone", String, self.drone_callback)

        rospy.Subscriber("mapping_data", String, self.mapping_callback)

        rospy.Subscriber("/cockpit/waypoint", Pose, self.planning_callback)

        rospy.Subscriber(
            "planning_decision_data", String, self.planning_decision_callback
        )

        rospy.Subscriber("/odometry/home_coordinates", String, self.odometry_callback)

        self.drone_command_pub = rospy.Publisher("drone_node", String, queue_size=10)

        self._feedback = MoveFeedback
        self._result = MoveResult

        # Move Action Server
        # Initialised for providing MoveAction for planning node
        self.move_action_server = actionlib.SimpleActionServer(
            Control.MOVE_ACTION_NAMESPACE,
            PlanningMoveAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )

        # Planning Command Server
        # Initialised for providing PlanningCommandAction for planning node
        self.planning_command_server = actionlib.SimpleActionServer(
            Control.COMMAND_ACTION_NAMESPACE,
            PlanningCommandAction,
            execute_cb=self.planning_decision_callback,
            auto_start=False,
        )

        self.drone_move_command = DroneMoveCommand()

        self.take_off_land_handler = TakeOffAndLandHandler()

        # For queuing the targets in PlanningMoveAction
        self.target = []

    def mapping_callback(self, mapping_data):
        """Returns the data from the mapping node

        @param mapping_data:
        """
        self.prev_x = mapping_data.x
        self.prev_y = mapping_data.y

    def drone_callback(self, data):
        """Returns the data from the drone node

        @param data:
        """
        self.drone_data = data.data

    def planning_decision_callback(self, goal):
        """Returns the decision data from the planning node if there is a need for
            landing or taking off

        @param goal
        @rtype: object
        """

        command = goal.command

        if command == TelloCommands.TAKEOFF:
            self.take_off_land_handler.handle_takeoff()
        elif command == TelloCommands.LAND:
            self.take_off_land_handler.handle_land()

        result = PlanningCommandResult()
        result.command_executed = True

        self.planning_command_server.set_succeeded(result)

    def planning_callback(self, planning_data):
        rospy.loginfo(f"Received planning_data: {planning_data}")
        self.target_x = planning_data.position.x
        self.target_y = planning_data.position.y

    def odometry_callback(self, msg):
        """Return th odometry data for orientation and position

        @param msg:
        """
        position = msg.pose.position
        orientation = msg.pose.orientation

        # TODO: Use this data for calculate_rotation_and_translation?

        x = position.x
        y = position.y
        z = position.z
        direction = orientation.z

    def calculate_rotation_and_translation(self, prev, target):
        """Calculates the rotation angle and translation for the drone node
        Return the angle in degrees and the translation as float

        @rtype: object
        """
        if prev.x is None or prev.y is None or target.x is None or target.y is None:
            degree_angle = 0.0
            distance = 0.0
        else:
            dx = target.x - prev.x
            dy = target.y - prev.y
            # hypoth = math.sqrt(dx ** 2 + dy ** 2)
            # sine = dy / hypoth
            # rad_angle = math.asin(sine)
            angle = math.atan2(dy, dx)
            degree_angle = math.degrees(angle)
            distance = float(math.sqrt(dx**2 + dy**2))

            self.prev = target

            rospy.loginfo(
                "Calculated distance: %.2f "
                ";"
                "Calculated degree angle: %.2f" % (distance, degree_angle)
            )

        return degree_angle, distance

    def execute_cb(self, goal):
        """Provides the calculated rotation and translation values as action messages to
        the drone node

        @rtype: object
        """

        result = False

        target_points = goal.target

        for target in target_points:

            # course angle and distance fpr movement are calculated
            course, distance = self.calculate_rotation_and_translation(self.prev, target)

            bear = course - self.prev_course

            # Angle is normalized for holding area in specific area
            bear = self.normalize_angle(bear)

            rospy.loginfo(
                "x1: %.2f y1: %.2f x2: %.2f y2: %.2f course: %.2f bear: %.2f dist: %.2f"
                % (self.prev.x, self.prev.y, target.x, target.y, course, bear, distance)
            )

            self.prev_course = course

            # Now the move command is send to the drone node
            if bear != 0.0:
                while True:
                    result = self.drone_move_command.move_drone(0.0, 0.0, 0.0, bear)
                    rospy.loginfo(result)
                    if result.success:
                        rospy.loginfo("Rotation was successful")
                        break
                    else:
                        rospy.loginfo("Retrying to rotate")
                        time.sleep(1)

            while True:
                result = self.drone_move_command.move_drone(distance, 0.0, 0.0, 0.0)
                rospy.loginfo(result)
                if result.success:
                    rospy.loginfo("Translation was successful")
                    break
                else:
                    rospy.loginfo("Retrying to translate")
                    time.sleep(1)

            # Send feedback via move action server for planning node
            # TODO: is feedback needed?
            #feedback = PlanningMoveFeedback()
            #feedback.progress = result.progress
            #self.move_action_server.publish_feedback(feedback)

        # Set result for move action server for planning node
        result_msg = PlanningMoveResult()
        result_msg.success = result.success
        self.move_action_server.set_succeeded(result_msg)

    def normalize_angle(self, angle):
        """Function which holds the angle in an area between -180 degrees
            and 180 degrees

        @param angle:
        @return:
        """
        if angle > 180.0:
            angle -= 360.0
        elif angle < -180.0:
            angle += 360.0
        return angle

    def main_process(self):
        """The function where several functions run in a loop and are executed

        @rtype: object
        """

        # Start the action server to provide MoveAction messages
        self.move_action_server.start()
        rospy.loginfo("move action server created")

        # Start command server to provide CommandAction messages
        self.planning_command_server.start()
        rospy.loginfo("planning command server created")

        # 10 Hz Rate
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():

            rate.sleep()


if __name__ == "__main__":
    try:
        control_node = DroneControl()
        control_node.main_process()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
