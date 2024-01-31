#!/usr/bin/env python3

import rospy  # the library should be added as package dependency for the package on which working here
from std_msgs.msg import Bool
from std_msgs.msg import Int32

BATTERY_FULL_PERCENTAGE = 100
BATTERY_THRESHOLD_IN_PERCENT = 10
BATTERY_DEFAULT_RATE = 0.20  # Send every 5 seconds


class BatterySubscriber:

    def start(self):
        pub = rospy.Publisher('/battery/return_signal', Bool, queue_size=10) # Bool is the message type
        rate = rospy.Rate(self._rate)
        message = Bool() # create a message
        while not rospy.is_shutdown():
            message.data = True if self._battery_status < BATTERY_THRESHOLD_IN_PERCENT else False
            pub.publish(message)
            rospy.loginfo("Publishing %s return signal", message.data)
            rate.sleep()  # wait according to the publishing rate

    def battery_status_callback(self, data: Int32): # never call this func. yourself, called when msg arrives
        rospy.loginfo("battery at %s%%", data.data)
        self._battery_status = data.data # keep as short as possible, since interrupts the execution flow

    def __init__(self, rate=BATTERY_DEFAULT_RATE):
        self._battery_status = BATTERY_FULL_PERCENTAGE # convention: prefix single underscore to non-publ. instan. vars
        self._sub = rospy.Subscriber('/drone/battery', Int32, callback=self.battery_status_callback)
        self._rate = rate


if __name__ == '__main__':
    try:
        rospy.init_node('battery_node')  # register the node with roscore, allowing it to communicate with other nodes
        battery_subscriber = BatterySubscriber()
        battery_subscriber.start()
    except rospy.ROSInterruptException:
        pass
