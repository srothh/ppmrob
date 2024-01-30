#!/usr/bin/env python3

import rospy  # the library should be added as package dependency for the package on which working here
from std_msgs.msg import Bool
from std_msgs.msg import Int32

BATTERY_FULL_PERCENTAGE = 100
BATTERY_THRESHOLD_IN_PERCENT = 10
BATTERY_DEFAULT_RATE = 0.20  # Send every 5 seconds


class BatterySubscriber:

    def start(self):
        pub = rospy.Publisher('/drone-control/return_signal', Bool, queue_size=10)
        rate = rospy.Rate(self._rate)
        while not rospy.is_shutdown():
            message = Bool()
            if self._battery_status < BATTERY_THRESHOLD_IN_PERCENT:
                message.data = True
            else:
                message.data = False
            pub.publish(message)
            rospy.loginfo("Send %s to /drone-control/return_signal", message.data)
            rate.sleep()  # wait according to the publishing rate

    def battery_status_callback(self, data):
        rospy.loginfo("battery at %s%%", data.data)
        self._battery_status = data.data

    def __init__(self, rate=BATTERY_DEFAULT_RATE):
        self._battery_status = BATTERY_FULL_PERCENTAGE
        self._sub = rospy.Subscriber('/battery', Int32, callback=self.battery_status_callback)
        self._rate = rate


if __name__ == '__main__':
    try:
        rospy.init_node('battery_node')  # start node
        battery_subscriber = BatterySubscriber()
        battery_subscriber.start()
    except rospy.ROSInterruptException:
        pass
