#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import TwistStamped, Twist
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Header
import tf2_ros
from geometry_msgs.msg import TransformStamped
import numpy as np
import common.config.defaults as defaults

ODOMETRY_DEFAULT_RATE = 10


class Odometry:
    def __init__(self, rate=ODOMETRY_DEFAULT_RATE):

        self._sub = rospy.Subscriber(
            defaults.drone_twist_sensor_publish_topic_name, TwistStamped, callback=self.read_vel
        )
        self._pub = rospy.Publisher(
            defaults.Odometry.WORLD_POSITION_TOPIC_NAME, PoseStamped, queue_size=10
        )
        self._tf = tf2_ros.TransformBroadcaster()
        self._rate = rospy.Rate(rate)
        self._messages = []

        # self._sens_data = Twist()
        # self._pose = Pose()
        # self._pose.orientation.w = 1
        # self._dt = 1/rate

        while not rospy.is_shutdown():
            try:
                pose = self.update_odom()
                # rospy.loginfo(self._pose)
                self.send_pose(pose)
                self.send_tf(pose)
                self._rate.sleep()
            except Exception as e:
                rospy.loginfo(e)

    def update_odom(self):
        pose = Pose()
        pose.orientation.w = 1
        # Integrate velocities to obtain the position
        times = []
        vx = []
        vy = []
        vz = []
        yaw = 0
        messages = self._messages.copy()
        for mess in messages:
            # queue size of twiststamped should be = 2
            times.append(mess.header.stamp.to_sec())
            vx.append(mess.twist.linear.x)
            vy.append(mess.twist.linear.y)
            vz.append(mess.twist.linear.z)
        if len(times) > 2:
            deltaTimeS = np.diff(times)
            pose.position.x = sum(vx[1 : len(vx)] * deltaTimeS) * (-10)
            pose.position.y = sum(vy[1 : len(vy)] * deltaTimeS) * (-10)
            pose.position.z = sum(vz[1 : len(vz)] * deltaTimeS) * 10

            last = messages[-1]
            pitch_rad = math.radians(last.twist.angular.x)
            roll_rad = math.radians(last.twist.angular.y)
            yaw_rad = math.radians(last.twist.angular.z)

            # Compute quaternion components
            cy = math.cos(yaw_rad * 0.5)
            sy = math.sin(yaw_rad * 0.5)
            cp = math.cos(pitch_rad * 0.5)
            sp = math.sin(pitch_rad * 0.5)
            cr = math.cos(roll_rad * 0.5)
            sr = math.sin(roll_rad * 0.5)
            pose.orientation.w = cy * cp * cr + sy * sp * sr
            pose.orientation.x = cy * cp * sr - sy * sp * cr
            pose.orientation.y = sy * cp * sr + cy * sp * cr
            pose.orientation.z = sy * cp * cr - cy * sp * sr

        return pose

    def read_vel(
        self, data: TwistStamped
    ):  # Never call this func. yourself, called when msg arrives
        self._messages.append(data)

    def send_pose(self, pose):
        msg = PoseStamped()
        msg.header = Header()
        msg.header.frame_id = "position"
        msg.header.stamp = rospy.Time.now()
        msg.pose = pose

        self._pub.publish(msg)

    def send_tf(self, pose):  # Function used to visualize
        # Publish the odometry TF data
        odom_tf = TransformStamped()
        odom_tf.header.stamp = rospy.Time.now()
        odom_tf.header.frame_id = "odom"
        odom_tf.child_frame_id = "base_link"
        odom_tf.transform.translation = pose.position
        odom_tf.transform.rotation = (
            pose.orientation
        )  # Rotation (Quaternion, with w=1.0 indicating no rotation)

        self._tf.sendTransform(odom_tf)


if __name__ == "__main__":
    try:
        rospy.init_node(
            "odometry"
        )  # Register the node with roscore, allowing it to communicate with other nodes
        odometry_subscriber = Odometry()
    except rospy.ROSInterruptException:
        pass
