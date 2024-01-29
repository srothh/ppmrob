import rospy
from djitellopy import Tello, TelloException


class KeepaliveServer:

    def __init__(self, drone):
        self._drone = drone

    def keepalive(self, event = None):
        try:
            result = self._drone.send_keepalive()
            if result == None:
                rospy.loginfo("drone dead?")
                # TODO: soutdown drone
        except TelloException:
            pass 
        