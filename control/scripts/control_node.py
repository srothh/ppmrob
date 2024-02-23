#!/usr/bin/env python3
import rospy
from control.scripts.action.drone_action_command import DroneActionCommand
from std_msgs.msg import String


class DroneControl:
    def __init__(self):
        # Class Variables from different nodes
        self.drone_data = None
        self.planning_data = None
        self.mapping_data = None
        rospy.init_node("drone_control_node", anonymous=True)

        rospy.Subscriber("mapping_data", String, self.cv_callback)

        rospy.Subscriber("planning_data", String, self.planning_callback)

        self.drone_command_pub = rospy.Publisher("drone_node", String, queue_size=10)

        self.drone_action_command = DroneActionCommand()

    def cv_callback(self, data):
        self.cv_data = data.data

    def planning_callback(self, planning_data):
        for point in planning_data:
            x1, y1, x2, y2, course, bear, dist = point
            if bear == 0:
                self.drone_action_command.send_action_command(dist, 0.0, 0.0, 0.0, 0.0, 0.0)
            elif bear != 0:
                self.drone_action_command.send_action_command(0.0, 0.0, 0.0, 0.0, 0.0, bear)

            ##TODO: Hier weitere Bedingunge hinzufügen


    def main_process(self):
        # 10 Hz Rate
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():

            ## TODO: Implementation of the logic for executing control data to tello

            if self.mapping_data is not None:
                rospy.loginfo(f"Received cv_data: {self.cv_data}")

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
