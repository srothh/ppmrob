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

        self.drone_action_command = DroneActionCommand()

    def mapping_callback(self, mapping_data):
        self.prev_x = mapping_data.x1
        self.prev_y = mapping_data.x1

    def drone_callback(self, data):
        self.drone_data = data.data

    def planning_callback(self, planning_data):
        self.target_x = planning_data.x2
        self.target_y = planning_data.y2

    def calculate_rotation_and_translation(self):
        if self.prev_x is None or self.prev_y is None or self.target_x is None or self.target_y is None:
            degree_angle = 0.0
            distance = 0
        else:
            dx = abs(self.target_x - self.prev_x)
            dy = abs(self.target_y - self.prev_y)
            hypoth = math.sqrt(dx ** 2 + dy ** 2)
            sine = dy / hypoth
            rad_angle = math.asin(sine)
            degree_angle = math.degrees(rad_angle)
            distance = dy

        return degree_angle, distance

    def execute_cb(self):
        degree_angle, distance = self.calculate_rotation_and_translation()
        if degree_angle == 0.0:
            self.drone_action_command.send_action_command(distance, 0.0, 0.0, 0.0, 0.0, 0.0)
        else:
            self.drone_action_command.send_action_command(0.0, 0.0, 0.0, 0.0, 0.0, degree_angle)

    def main_process(self):
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
