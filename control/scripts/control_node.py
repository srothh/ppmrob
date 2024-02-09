#!/usr/bin/env python3
import rospy
from std_msgs.msg import String


class DroneControl:
    def __init__(self):
        # Class Variables from different nodes
        self.battery_data = None
        self.planning_data = None
        self.cv_data = None

    def __int__(self):
        rospy.init_node("drone_control_node", anonymous=True)

        rospy.Subscriber("cv_data", String, self.cv_callback)

        rospy.Subscriber("planning_data", String, self.planning_callback)

        rospy.Subscriber("battery_data", String, self.battery_callback)

        self.tello_command_pub = rospy.Publisher("tello_data", String, queue_size=10)

    def cv_callback(self, data):
        self.cv_data = data.data

    def planning_callback(self, data):
        self.planning_data = data

    def battery_callback(self, data):
        self.battery_data = data

    def main_process(self):
        # 10 Hz Rate
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():

            ## TODO: Implementation of the logic for executing control data to tello

            if self.cv_data is not None:
                rospy.loginfo(f"Received cv_data: {self.cv_data}")

            if self.planning_data is not None:
                rospy.loginfo(f"Received planning_data: {self.planning_data}")

            if self.battery_data is not None:
                rospy.loginfo(f"Received battery_data: {self.battery_data}")

            self.tello_command_pub.publish("example_command")

            rate.sleep()


if __name__ == '__main__':
    try:
        control_node = DroneControl()
        control_node.main_process()
    except rospy.ROSInterruptException:
        pass
