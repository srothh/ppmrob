#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point

import common.config.defaults as defaults  # TODO add to dockerfile as per issue!

def publish_drone_position():
    rospy.init_node('mock_drone_position_publisher', anonymous=True)
    pub = rospy.Publisher(defaults.Control.WORLD_POSITION_TOPIC_NAME, Point, queue_size=10)
    rate = rospy.Rate(1)  # Publish at 1 Hz

    while not rospy.is_shutdown():
        # Create a dummy Point representing the drone's world position
        position = Point()
        position.x = 1.0  # Example x position
        position.y = 2.0  # Example y position
        position.z = 3.0  # Example z position (altitude)

        # Publish the Point
        pub.publish(position)
        rospy.loginfo(f"Published mock drone position: ({position.x}, {position.y}, {position.z})")
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_drone_position()
    except rospy.ROSInterruptException:
        pass