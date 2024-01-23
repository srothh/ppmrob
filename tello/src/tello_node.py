#!/usr/bin/env python3

import rospy
from std_msgs.msg import String


def tello_node():
    # Initialize the ROS node
    rospy.init_node('tello', anonymous=True)
    print("starting tello node")
    # start action servers
    #launch_server = LaunchAction("launch")
    #move_server = MoveAction("move")
    #command_server = CommandAction("command")

    rospy.is_spin()
    
if __name__ == '__main__':
    try:
        tello_node()
    except rospy.ROSInterruptException:
        pass
