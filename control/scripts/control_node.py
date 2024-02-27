#!/usr/bin/env python3
import math

import rospy
from control.scripts.action.drone_action_command import DroneActionCommand
from std_msgs.msg import String


class DroneControl:
    def __init__(self):
        # Class Variables from different nodes
        self.prev_x = None
        self.prev_y = None
        self.drone_data = None
        self.planning_data = None
        self.mapping_data = None
        rospy.init_node("control_node_listener", anonymous=True)

        rospy.Subscriber("drone", String, self.drone_callback)

        rospy.Subscriber("mapping_data", String, self.mapping_callback)

        rospy.Subscriber("planning_data", String, self.planning_callback)

        self.drone_command_pub = rospy.Publisher("drone_node", String, queue_size=10)

        self.drone_action_command = DroneActionCommand()

    def mapping_callback(self, data):
        self.mapping_data = data.data

    def drone_callback(self, data):
        self.drone_data = data.data

    def planning_callback(self, planning_data):
        for point in planning_data:
            x1, y1 = point
            degree_angle, distance = self.calculate_rotation_and_translation(x1, y1)
            if degree_angle == 0.0:
                self.drone_action_command.send_action_command(distance, 0.0, 0.0, 0.0, 0.0, 0.0)
            else:
                self.drone_action_command.send_action_command(0.0, 0.0, 0.0, 0.0, 0.0, degree_angle)

            ##TODO: Hier evtl weitere Bedingunge hinzufügen

    def calculate_rotation_and_translation(self, current_x, current_y):
        if self.prev_x is None or self.prev_y is None:
            self.prev_x = current_x
            self.prev_y = current_y
            degree_angle = 0.0
            distance = 0
        else:
            dx = abs(current_x - self.prev_x)
            dy = abs(current_y - self.prev_y)
            hypoth = math.sqrt(dx ** 2 + dy ** 2)
            sine = dy / hypoth
            rad_angle = math.asin(sine)
            degree_angle = math.degrees(rad_angle)
            distance = dy

            self.prev_x = current_x
            self.prev_y = current_y
        return degree_angle, distance

    def main_process(self):
        # 10 Hz Rate
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():

            ## TODO: Implementation of the logic for executing control data to tello
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
