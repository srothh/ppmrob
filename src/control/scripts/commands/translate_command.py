#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose

class TranslateCommand(object):

    def translate(self, x, y, z, w):
        pose_msg = Pose()
        pose_msg.position.x = x
        pose_msg.position.y = y
        pose_msg.position.z = z

        pose_publisher = rospy.Publisher('/robot_pose', Pose, queue_size=10)

        pose_publisher.publish(pose_msg)

        rospy.sleep(1.0)

