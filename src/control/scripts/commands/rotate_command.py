#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose, Quaternion


class RotateCommand(object):

    def rotate(self, x, y, z, w):
        pose_message = Pose()
        quaternion = Quaternion()
        quaternion.x = x
        quaternion.y = y
        quaternion.z = z
        quaternion.w = w

        pose_message.orientation = quaternion

        pose_publisher = rospy.Publisher('/robot_pose', Pose, queue_size=10)

        rate = rospy.Rate(1)

        while not rospy.is_shutdown():
            pose_publisher.publish(pose_message)
            rate.sleep()
