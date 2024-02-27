import rospy  # the library should be added as package dependency for the package on which working here
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header



ODOMETRY_DEFAULT_RATE = 1  # Send every second
DEFAULT_POSITION = 0


class OdometrySubscriber:

    def start(self):
        pub = rospy.Publisher('/odometry/return_signal', PoseStamped, queue_size=10) # Bool is the message type
        rate = rospy.Rate(self._rate) # set rate of the following loop
        message = PoseStamped() # create a message for return signal
        while not rospy.is_shutdown():
            message.header = Header()
            message.header.frame_id = "position"
            message.header.stamp = rospy.Time.now()
            times = []
            vx = []
            vy = []
            vz = []
            for mess in self._messages:
                # queue size of twiststamped should be = 2
                times.append(mess.header.stamp.secs)
                vx.append(mess.twist.linear.x)
                vy.append(mess.twist.linear.y)
                vz.append(mess.twist.linear.z)
            message.pose.position.x = self._position_x + vx[0] / (times[1]-times[0])
            message.pose.position.y = self._position_y + vy[0] / (times[1]-times[0])
            message.pose.position.z = self._position_z + vz[0] / (times[1]-times[0])
            self._position_x = message.pose.position.x
            self._position_y = message.pose.position.y
            self._position_z = message.pose.position.z
            pub.publish(message) # publish the return signal
            #rospy.loginfo("Publishing %s return signal", message.data)
            rate.sleep()  # wait according to the publishing rate

    def odometry_callback(self, data: TwistStamped): # never call this func. yourself, called when msg arrives
        #rospy.loginfo("velocity: %s%%", data.
        self._messages = rospy.wait_for_message('/drone/twist', TwistStamped, timeout=rospy.Duration(0.1))  # Read all messages currently in the queue


    def __init__(self, rate=ODOMETRY_DEFAULT_RATE):
        self._position_x = DEFAULT_POSITION
        self._position_y = DEFAULT_POSITION
        self._position_z = DEFAULT_POSITION
        self._sub = rospy.Subscriber('/drone/twist', TwistStamped, callback=self.odometry_callback)
        self._rate = rate # convention: prefix single underscore to non-public instance variables


if __name__ == '__main__':
    try:
        rospy.init_node('odometry')  # register the node with roscore, allowing it to communicate with other nodes
        odometry_subscriber = OdometrySubscriber()
        odometry_subscriber.start()
    except rospy.ROSInterruptException:
        pass
