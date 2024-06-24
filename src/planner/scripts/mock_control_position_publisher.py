#!/usr/bin/env python3
import rospy
import actionlib
import py_trees
import py_trees_ros
from geometry_msgs.msg import Point, PoseStamped

import common.config.defaults as defaults


def publish_drone_position():
    rospy.init_node("mock_drone_position_publisher", anonymous=True)
    conf = rospy.get_param("~conf")
    if conf > 4:
        conf = 1
    pub = rospy.Publisher(
        defaults.Odometry.WORLD_POSITION_TOPIC_NAME, PoseStamped, queue_size=10
    )
    rate = rospy.Rate(1)  # Publish at 1 Hz
    rospy.loginfo("Started publishing position config: "+str(conf))
    while not rospy.is_shutdown():
        pos = PoseStamped()
        if conf == 1:
            pos.pose.position.x = 0.0
            pos.pose.position.y = 2.0
            pos.pose.position.z = 0.0
        elif conf == 2:
            pos.pose.position.x = 0.0
            pos.pose.position.y = 2.0
            pos.pose.position.z = 0.0
        elif conf == 3:
            pos.pose.position.x = 0.0
            pos.pose.position.y = 2.0
            pos.pose.position.z = 0.0
        elif conf == 4:
            pos.pose.position.x = 0.0
            pos.pose.position.y = 2.0
            pos.pose.position.z = 0.0

        # Publish the PoseStamped
        pub.publish(pos)
        rate.sleep()


if __name__ == "__main__":
    try:
        publish_drone_position()
    except rospy.ROSInterruptException:
        pass
