#!/usr/bin/env python3

import rospy
from std_msgs.msg import String


def publisher_node():
    # Initialize the ROS node
    rospy.init_node('publisher', anonymous=True)

    # Create a publisher for the 'chatter' topic
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        # Create and publish a message
        message = "Hello, ROS!"
        rospy.loginfo(message)
        pub.publish(message)

        # Wait according to the publishing rate
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher_node()
    except rospy.ROSInterruptException:
        pass
