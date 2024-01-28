#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
from djitellopy import Tello
import threading

class BatteryPublisher():

    def pub_thread(self):
        while not rospy.is_shutdown():
            # Create and publish a message
            message = Int32()
            message.data = self._drone.query_battery()
            rospy.loginfo(message)
            self._pub.publish(message)
            # Wait according to the publishing rate
            self._rate.sleep()

    def start(self):
        worker = threading.Thread(target=self.pub_thread)
        worker.start()


    def __init__(self, drone):
        self._pub = rospy.Publisher('battery', Int32, queue_size=10)
        self._rate = rospy.Rate(1)  # 1 Hz
        self._drone = drone



if __name__ == '__main__':
    try:
        rospy.init_node('battery')
        drone = Tello()
        drone.connect()
        b = BatteryPublisher(drone)
        b.start()
    except rospy.ROSInterruptException:
        pass
