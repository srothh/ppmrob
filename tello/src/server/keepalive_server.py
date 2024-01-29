class KeepaliveServer:

    def __init__(self, drone):
        self._drone = drone

    def keepalive(self, event = None):
        self._drone.send_keepalive()
        