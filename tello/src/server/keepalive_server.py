from djitellopy import Tello, TelloException


class KeepaliveServer:

    def __init__(self, drone):
        self._drone = drone

    def keepalive(self, event = None):
        try:
            self._drone.send_keepalive()
        except TelloException:
            pass 
        