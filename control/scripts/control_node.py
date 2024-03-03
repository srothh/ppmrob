#!/usr/bin/env python3
import math

import actionlib
import rospy
from control.scripts.action.drone_action_command import DroneActionCommand
from std_msgs.msg import String
import control.msg
from control.msg import DroneActionCommandAction, DroneActionCommandResult


class DroneControl:
    def __init__(self):
        # Class Variables from different nodes
        self.prev_x = None
        self.prev_y = None
        self.target_x = None
        self.target_y = None
        self.course = None
        self.drone_data = None
        self.planning_data = None
        self.mapping_data = None
        rospy.init_node("control_node_listener", anonymous=True)

        rospy.Subscriber("drone", String, self.drone_callback)

        rospy.Subscriber("mapping_data", String, self.mapping_callback)

        rospy.Subscriber("planning_data", String, self.planning_callback)

        self.drone_command_pub = rospy.Publisher("drone_node", String, queue_size=10)

        _feedback = control.msg.CommandFeedback()
        _result = control.msg.CommandResult()

        # Action Server
        self.server = actionlib.SimpleActionServer(
            'Movement-Server',
            control.msg.DroneActionCommandAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )

        self.drone_action_command = DroneActionCommand()

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

    def planning_callback(self, planning_data):
        """ Returns the data from the planning node

        @param planning_data
        @rtype: object
        """
        self.target_x = planning_data.x2
        self.target_y = planning_data.y2

    def calculate_rotation_and_translation(self):
        """ Calculates the rotation angle and translation for the drone node
        Return the angle in degrees and the translation as float

        @rtype: object
        """
        if self.prev_x is None or self.prev_y is None or self.target_x is None or self.target_y is None:
            degree_angle = 0.0
            distance = 0.0
        else:
            dx = abs(self.target_x - self.prev_x)
            dy = abs(self.target_y - self.prev_y)
            # hypoth = math.sqrt(dx ** 2 + dy ** 2)
            # sine = dy / hypoth
            # rad_angle = math.asin(sine)
            angle = math.atan2(dy, dx)
            degree_angle = math.degrees(angle)
            distance = float(dy)

        return degree_angle, distance

    def execute_cb(self):
        """ Sends the calculated rotation and translation values as action messages to
        the drone node

        @rtype: object
        """
        degree_angle, distance = self.calculate_rotation_and_translation()
        if degree_angle == 0.0:
            self.drone_action_command.send_action_command(distance, 0.0, 0.0, 0.0, 0.0, 0.0)
        else:
            self.drone_action_command.send_action_command(0.0, 0.0, 0.0, 0.0, 0.0, degree_angle)

        result = DroneActionCommandResult()
        result.success = True
        self.server.set_succeeded(result)

    def main_process(self):
        """ The function where several functions run in a loop and are executed

        @rtype: object
        """

        # Start the action server
        self.server.start()

        # 10 Hz Rate
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():

            self.execute_cb()

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
