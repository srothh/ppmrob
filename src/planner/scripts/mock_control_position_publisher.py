#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point

import common.config.defaults as defaults  # TODO add to dockerfile as per issue!


def publish_drone_position():
    rospy.init_node("mock_drone_position_publisher", anonymous=True)
    pub = rospy.Publisher(
        defaults.Control.WORLD_POSITION_TOPIC_NAME, Point, queue_size=10
    )
    rate = rospy.Rate(1)  # Publish at 1 Hz

    while not rospy.is_shutdown():
        position = Point()
        position.x = 75.0  # Example x position in cm
        position.y = 2.0  # Example y position in cm
        position.z = 255.0  # Example z position in cm

        # Publish the Point
        pub.publish(position)
        rate.sleep()


if __name__ == "__main__":
    try:
        publish_drone_position()
    except rospy.ROSInterruptException:
        pass
