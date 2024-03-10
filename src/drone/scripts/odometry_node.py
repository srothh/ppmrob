#!/usr/bin/env python3

import rospy  # the library should be added as package dependency for the package on which working here
import numpy as np
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import common.config.defaults



ODOMETRY_DEFAULT_RATE = 10  # in Hz
DEFAULT_POSITION = 0


class OdometrySubscriber:

    _messages = []

    def start(self):
        pub = rospy.Publisher('/odometry/return_signal', PoseStamped, queue_size=10) 
        rate = rospy.Rate(self._rate) # set rate of the following loop
        message = PoseStamped() # create a message for return signal
        while not rospy.is_shutdown():
            try:
                message.header = Header()
                message.header.frame_id = "position"
                message.header.stamp = rospy.Time.now()
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
                if (len(times) > 2):
                    deltaTimeS=np.diff(times)
                    message.pose.position.x =  sum(vx[1:len(vx)]*deltaTimeS)*(10)
                    message.pose.position.y =  sum(vy[1:len(vy)]*deltaTimeS)*(-10)
                    message.pose.position.z =  sum(vz[1:len(vz)]*deltaTimeS)*10

                    #calculate azimuth
                    message.pose.orientation.z = (messages[-1].twist.angular.z + 360) % 360
                    pub.publish(message) # publish the return signal
                    #self._messages = []
                    #rospy.loginfo("Publishing %s return signal", message.data)
                rate.sleep()  # wait according to the publishing rate
            except Exception as e:
                rospy.loginfo(e)
    def odometry_callback(self, data: TwistStamped): # never call this func. yourself, called when msg arrives
        #rospy.loginfo("velocity: %s" % data)
        #self._messages = rospy.wait_for_message(common.config.defaults.drone_twist_sensor_publish_topic_name, TwistStamped)  # Read all messages currently in the queue
        self._messages.append(data)
        #rospy.loginfo("messages: %s", self._messages)



    def __init__(self, rate=ODOMETRY_DEFAULT_RATE):
        self._position_x = DEFAULT_POSITION
        self._position_y = DEFAULT_POSITION
        self._position_z = DEFAULT_POSITION
        self._sub = rospy.Subscriber(common.config.defaults.drone_twist_sensor_publish_topic_name, TwistStamped, callback=self.odometry_callback)
        self._rate = rate # convention: prefix single underscore to non-public instance variables


if __name__ == '__main__':
    try:
        rospy.init_node('odometry')  # register the node with roscore, allowing it to communicate with other nodes
        odometry_subscriber = OdometrySubscriber()
        odometry_subscriber.start()
    except rospy.ROSInterruptException:
        pass
