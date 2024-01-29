#!/usr/bin/env python3

import rospy
from tello.msg import Telemetry
from djitellopy import Tello

class TelemetrySensor():

    # publish state data
    def publish(self, event=None):
             if self._drone.get_current_state():
                msg = Telemetry()
                # attributes of msg have same names as the tello.state dictonary keys)
                for field in [a for a in dir(msg) if not a.startswith('_') and not callable(getattr(msg, a))]:
                    setattr(msg, field, self._drone.get_state_field(field))
                rospy.logdebug(msg)
                self._publisher.publish(msg)


    def __init__(self, drone):
        self._publisher = rospy.Publisher('telemetry', Telemetry, queue_size=10)
        self._drone = drone
        rospy.loginfo('TelemetrySensor initalized')

if __name__ == '__main__':
    try:
        rospy.init_node('telemetry')
        drone = Tello()
        drone.connect()
        sensor = TelemetrySensor(drone)
        rospy.Timer(rospy.Duration(1.0), sensor.publish)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
