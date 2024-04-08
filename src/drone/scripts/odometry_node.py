#!/usr/bin/env python3

import rospy  
import math 
from geometry_msgs.msg import TwistStamped, Twist
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Header
import tf2_ros
from geometry_msgs.msg import TransformStamped

ODOMETRY_DEFAULT_RATE = 100  

class Odometry:
    def __init__(self, rate=ODOMETRY_DEFAULT_RATE):
        
        self._sub = rospy.Subscriber("/drone/twist", TwistStamped, callback=self.read_vel)
        self._pub = rospy.Publisher('/odometry/return_signal', PoseStamped, queue_size=10) 
        self._tf = tf2_ros.TransformBroadcaster()
        self._rate = rospy.Rate(rate) 
    
    
        self._sens_data = Twist()
        self._pose = Pose()
        self._pose.orientation.w = 1
        self._dt = 1/rate

        while not rospy.is_shutdown():
            try:
                self.update_odom()
                #rospy.loginfo(self._pose)
                self.send_pose()
                self.send_tf()
                self._rate.sleep()
            except Exception as e:
                rospy.loginfo(e)

    def update_odom(self):

        # Integrate velocities to obtain the position
        self._pose.position.x += self._sens_data.linear.x*self._dt
        self._pose.position.y += self._sens_data.linear.y*self._dt
        self._pose.position.z += self._sens_data.linear.z*self._dt
        pitch_rad = math.radians(self._sens_data.angular.x)
        roll_rad = math.radians(self._sens_data.angular.y)
        yaw_rad = math.radians(self._sens_data.angular.z)

        # Compute quaternion components
        cy = math.cos(yaw_rad * 0.5)
        sy = math.sin(yaw_rad * 0.5)
        cp = math.cos(pitch_rad * 0.5)
        sp = math.sin(pitch_rad * 0.5)
        cr = math.cos(roll_rad * 0.5)
        sr = math.sin(roll_rad * 0.5)
        self._pose.orientation.w = cy * cp * cr + sy * sp * sr
        self._pose.orientation.x = cy * cp * sr - sy * sp * cr
        self._pose.orientation.y = sy * cp * sr + cy * sp * cr
        self._pose.orientation.z = sy * cp * cr - cy * sp * sr

    def read_vel(self, data: TwistStamped): # Never call this func. yourself, called when msg arrives
        self._sens_data = data.twist

    def send_pose(self):     
        msg = PoseStamped()
        msg.header = Header()
        msg.header.frame_id = "position"
        msg.header.stamp = rospy.Time.now()
        msg.pose = self._pose
        
        self._pub.publish(msg)

    def send_tf(self): # Function used to visualize
        # Publish the odometry TF data
        odom_tf = TransformStamped()
        odom_tf.header.stamp = rospy.Time.now()
        odom_tf.header.frame_id = "odom"
        odom_tf.child_frame_id = "base_link"  
        odom_tf.transform.translation = self._pose.position
        odom_tf.transform.rotation = self._pose.orientation  # Rotation (Quaternion, with w=1.0 indicating no rotation)
        
        self._tf.sendTransform(odom_tf)


if __name__ == '__main__':
    try:
        rospy.init_node('odometry')  # Register the node with roscore, allowing it to communicate with other nodes
        odometry_subscriber = Odometry()
    except rospy.ROSInterruptException:
        pass
