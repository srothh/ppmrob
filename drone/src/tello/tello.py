import socket, select
import time
import rospy
from threading import Thread, Lock
from collections import deque
import av
import numpy as np
from typing import Optional, Union, Type, Dict


class Tello:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tello_address = ('192.168.10.1', 8889)
    locaddr = ('',8889) 

    # VideoCapture object
    background_frame_read: Optional['BackgroundFrameRead'] = None


    def __init__(self):
        self.sock.setblocking(0)
        self.sock.bind(self.locaddr)
        

        

    def connect(self):
        self.connected = False
        # block till drone is connected
        while not self.connected:
            self.connected = self.command('command', timeout=1) 
            time.sleep(1)
        
        self.connected = True
        rospy.loginfo("connected, battery: %s" % self.send_command_with_return('battery?'))

    ## blocking method to send command and return with boolen success
    def command(self, cmd, timeout=10) -> bool:
        result = False
        # send command
        self.send_command(cmd)

        # endless loop monitorig socket with select
        counter = 0
        while(True):
            ready = select.select([self.sock], [], [], 1.0)
            if ready[0]:
                data, server = self.sock.recvfrom(1024)
                try:
                    print(data.decode(encoding="utf-8"))
                    result = True if data.decode(encoding="utf-8") == "ok" else False
                except Exception:
                    print('could not decode received data')
                break
            else:
                #print(counter)
                print ('.', end='', flush=True)
            if counter > timeout:
                print ('\n timeout')
                result = False                    
                break
            counter += 1
        return result

    ## blocking method to send command and return with the string result
    def send_command_with_return(self, cmd, timeout=10) -> str:
        result = ''
        self.send_command(cmd)
        start_time = time.time()
        while time.time() - start_time < timeout:
            ready = select.select([self.sock], [], [], 0.3)
            if ready[0]:
                result = self.sock.recv(1024)
                break
            else:
                print ('.', end='', flush=True)
        return result.decode(encoding="utf-8")

    def end(self):
        self.sock.close()



    ## send command to drone and return imidiately
    def send_command(self, cmd):
        sent = self.sock.sendto(cmd.encode(encoding="utf-8"), self.tello_address)
        return sent
    def execute_commands(self, cmds):
        for cmd in cmds:
            self.command(cmd)
            time.sleep(1)
        return True

    #image frame getter copied from dijtello lib
    def get_frame_read(self, with_queue = False, max_queue_len = 32) -> 'BackgroundFrameRead':
            """Get the BackgroundFrameRead object from the camera drone. Then, you just need to call
            backgroundFrameRead.frame to get the actual frame received by the drone.
            Returns:
                BackgroundFrameRead
            """
            if self.background_frame_read is None:
                address_schema = 'udp://@{ip}:{port}'  # + '?overrun_nonfatal=1&fifo_size=5000'
                address = address_schema.format(ip='', port=11111)
                self.background_frame_read = BackgroundFrameRead(self, address, with_queue, max_queue_len)
                self.background_frame_read.start()
            return self.background_frame_read


class BackgroundFrameRead:
    """
    This class read frames using PyAV in background. Use
    backgroundFrameRead.frame to get the current frame.
    """

    def __init__(self, tello, address, with_queue = False, maxsize = 32):
        self.address = address
        self.lock = Lock()
        self.frame = np.zeros([300, 400, 3], dtype=np.uint8)
        self.frames = deque([], maxsize)
        self.with_queue = with_queue

        # Try grabbing frame with PyAV
        # According to issue #90 the decoder might need some time
        # https://github.com/damiafuentes/DJITelloPy/issues/90#issuecomment-855458905
        try:
            self.container = av.open(self.address, timeout=(5, None))
        except av.error.ExitError:
            raise Exception('Failed to grab video frames from video stream')

        self.stopped = False
        self.worker = Thread(target=self.update_frame, args=(), daemon=True)

    def start(self):
        """Start the frame update worker
        Internal method, you normally wouldn't call this yourself.
        """
        self.worker.start()

    def update_frame(self):
        """Thread worker function to retrieve frames using PyAV
        Internal method, you normally wouldn't call this yourself.
        """
        try:
            for frame in self.container.decode(video=0):
                if self.with_queue:
                    self.frames.append(np.array(frame.to_image()))
                else:
                    self.frame = np.array(frame.to_image())

                if self.stopped:
                    self.container.close()
                    break
        except av.error.ExitError:
            raise Exception('Do not have enough frames for decoding, please try again or increase video fps before get_frame_read()')

    def get_queued_frame(self):
        """
        Get a frame from the queue
        """
        with self.lock:
            try:
                return self.frames.popleft()
            except IndexError:
                return None

    @property
    def frame(self):
        """
        Access the frame variable directly
        """
        if self.with_queue:
            return self.get_queued_frame()

        with self.lock:
            return self._frame

    @frame.setter
    def frame(self, value):
        with self.lock:
            self._frame = value

    def stop(self):
        """Stop the frame update worker
        Internal method, you normally wouldn't call this yourself.
        """
        self.stopped = True
