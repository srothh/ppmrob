#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion, Vector3
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class ImuSensor():

    _drone = None

    def __init__(self, topic='/drone/imu', drone=None):
        self._publish_to_topic = topic
        self._publisher = rospy.Publisher(self._publish_to_topic, Imu, queue_size=10)
        self._drone = drone        
        self._counter = 0
        rospy.loginfo('ImuSensor initalized')

    def publish(self, event=None, state=None):
        try:
            #rospy.loginfo("Publishing TwistStamped from %s" % state)
            if state:
                msg = Imu()
                msg.header = Header()
                msg.header.frame_id = "map"
                msg.header.stamp = rospy.Time.now()
                
                q = quaternion_from_euler(state.get('roll'), state.get('pitch'), state.get('yaw'))
                msg.orientation = Quaternion(q[0], q[1], q[2], q[3])

                msg.orientation_covariance = [0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0]
               
                angular_velocity = Vector3()
                angular_velocity_covariance = [-1.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0]

                msg.linear_acceleration = Vector3()
                msg.linear_acceleration.x = state.get('agx')
                msg.linear_acceleration.y = state.get('agy')
                msg.linear_acceleration.z = state.get('agz')
                msg.linear_acceleration_covariance = [0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0]

                self._publisher.publish(msg)
                self._counter += 1
        except Exception as e:
            rospy.loginfo(e)
    

