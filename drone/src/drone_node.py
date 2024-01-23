#!/usr/bin/env python3

import rospy
from std_msgs.msg import String


def drone_node():
    # Initialize the ROS node
    rospy.init_node('drone', anonymous=True)



    while not rospy.is_shutdown():
        print
    
if __name__ == '__main__':
    try:
        drone_node()
    except rospy.ROSInterruptException:
        pass
