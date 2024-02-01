import rospy
import time
from djitellopy import Tello, TelloException


class KeepaliveServer:

    def __init__(self, drone):
        self._drone = drone

    def keepalive(self, event = None):
        try:
            #check if last_received_command_timestamp is before 10 sec
            if self._drone.get_current_state() and time.time() - self._drone.last_received_command_timestamp  > 10:
                result = self._drone.send_command_with_return('keepalive')
                if result == None:
                    rospy.loginfo("drone dead?")
                    # TODO: shutdown drone
        except TelloException:
            pass 
        