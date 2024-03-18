#!/usr/bin/env python3
import math

import actionlib
import rospy
import control.msg
import commands
from action import DroneMoveCommand
from std_msgs.msg import String
from commands.takeoff_land_handler import TakeOffAndLandHandler
from geometry_msgs.msg import Transform, Vector3, Quaternion
from actionlib_msgs.msg import GoalStatus
from control.msg import MoveAction, MoveResult, MoveFeedback, PlanningMoveAction, PlanningMoveResult, PlanningCommandAction


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
        self.prev_course = None
        self.drone_data = None
        self.planning_data = None
        self.mapping_data = None
        rospy.init_node("control_node", anonymous=True)

        rospy.Subscriber("drone", String, self.drone_callback)

        rospy.Subscriber("mapping_data", String, self.mapping_callback)

        rospy.Subscriber("planning_decision_data", String, self.planning_decision_callback)

        rospy.Subscriber("odometry/return_signal", String, self.odometry_callback)

        self.drone_command_pub = rospy.Publisher("drone_node", String, queue_size=10)

        self._feedback = MoveFeedback
        self._result = MoveResult

        # Move Action Server
        # Initialised for providing MoveAction for planning node
        self.move_action_server = actionlib.SimpleActionServer(
            'Move-Action-Server',
            PlanningMoveAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )

        # Planning Command Server
        # Initialised for providing PlanningCommandAction for planning node
        self.planning_command_server = actionlib.SimpleActionServer(
            'Planning-Command-Server',
            PlanningCommandAction,
            execute_cb=self.planning_decision_callback,
            auto_start=False
        )

        self.drone_move_command = DroneMoveCommand()

        self.take_off_land_handler = TakeOffAndLandHandler()

    def mapping_callback(self, mapping_data):
        """ Returns the data from the mapping node

        @param mapping_data:
        """
        self.prev_x = mapping_data.x1
        self.prev_y = mapping_data.y1

    def drone_callback(self, data):
        """ Returns the data from the drone node

        @param data:
        """
        self.drone_data = data.data

    def planning_decision_callback(self, goal):
        """ Returns the decision data from the planning node if there is a need for
            landing or taking off

        @param goal
        @rtype: object
        """

        command = goal.command

        if command == 'Take off':
            self.take_off_land_handler.handle_takeoff(self.takeOff)
        elif command == 'Stop and Land':
            self.take_off_land_handler.handle_takeoff(self.land)

    def planning_callback(self, planning_data):
        self.target_x = planning_data.x2
        self.target_y = planning_data.y2

    def odometry_callback(self, msg):
        """ Return th odometry data for orientation and position

        @param msg:
        """
        position = msg.pose.position
        orientation = msg.pose.orientation

        #TODO: Use this data for calculate_rotation_and_translation?

        x = position.x
        y = position.y
        z = position.z
        direction = orientation.z

    def calculate_rotation_and_translation(self, prev, target):
        """ Calculates the rotation angle and translation for the drone node
        Return the angle in degrees and the translation as float

        @rtype: object
        """
        if prev.x is None or prev.y is None or target.x is None or target.y is None:
            degree_angle = 0.0
            distance = 0.0
        else:
            dx = abs(target.x - prev.x)
            dy = abs(target.y - prev.y)
            # hypoth = math.sqrt(dx ** 2 + dy ** 2)
            # sine = dy / hypoth
            # rad_angle = math.asin(sine)
            angle = math.atan2(dy, dx)
            degree_angle = math.degrees(angle)
            distance = float(dy)

            self.prev = target

        return degree_angle, distance

    def execute_cb(self, goal):
        """ Provides the calculated rotation and translation values as action messages to
        the drone node

        @rtype: object
        """

        target = goal.target

        # course angle and distance fpr movement are calculated
        course, distance = self.calculate_rotation_and_translation(self.prev, target)

        bear = course - self.prev_course

        # Angle is normalized for holding area in specific area
        bear = self.normalize_angle(bear)

        rospy.loginfo("x1: %.2f y1: %.2f x2: %.2f y2: %.2f course: %.2f bear: %.2f dist: %.2f" % (
            self.prev.x, self.prev.y, target.x, target.y, course, bear, distance))

        self.prev_course = course

        # Now the move command is send to the drone node
        if bear != 0.0:
            while True:
                result = self.drone_move_command.move_drone(0.0, 0.0, 0.0, bear)
                if result.success:
                    break
                else:
                    rospy.loginfo("Retrying to rotate")
        else:
            while True:
                result = self.drone_move_command.move_drone(distance, 0.0, 0.0, 0.0)
                if result.success:
                    break
                else:
                    rospy.loginfo("Retrying to translate")

        # Send feedback via move action server for planning node
        feedback = MoveFeedback()
        feedback.progress = result.progress
        self.move_action_server.publish_feedback(feedback)

        # Set result for move action server for planning node
        result_msg = PlanningMoveResult()
        result_msg.success = result.success
        self.move_action_server.set_succeeded(result_msg)

    def normalize_angle(self, angle):
        """ Function which holds the angle in an area between -180 degrees
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
        """ The function where several functions run in a loop and are executed

        @rtype: object
        """

        # Start the action server to provide MoveAction messages
        self.move_action_server.start()

        # 10 Hz Rate
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():

            # self.execute_cb()

            if self.drone_data is not None:
                rospy.loginfo(f"Received drone_data: {self.drone_data}")

            if self.mapping_data is not None:
                rospy.loginfo(f"Received mapping_data: {self.mapping_data}")

            if self.planning_data is not None:
                rospy.loginfo(f"Received planning_data: {self.planning_data}")

            rate.sleep()


if __name__ == '__main__':
    try:
        control_node = DroneControl()
        control_node.main_process()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
