import socket, select
import threading
import time
import rospy
from threading import Event, Thread, Lock
from collections import deque
import av
import numpy as np
from typing import Optional, Union, Type, Dict
import socketserver



class Tello:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tello_address = ('192.168.10.1', 8889)
    locaddr = ('',8889)

    state_port = 8890 
    state_recv_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    state_handlers = []

    # VideoCapture object
    background_frame_read: Optional['BackgroundFrameRead'] = None

    response_received = Event()
    response_queue = []
    command_queue = []

    response_thread = None
    state_thread = None

    _command_timeout = 10

    running = True

    last_command_ts = None

    def __init__(self, command_timeout = 10):
        
        self._command_timeout = command_timeout

        # only words with thread.select
        #self.sock.setblocking(0)
        self.sock.bind(self.locaddr)

        self.state_recv_socket.bind(('', self.state_port))
        

    # connect to drone
    # blocks till connection is made
    def connect(self) -> bool:
        self.connected = False
        
        response_thread = Thread(target = self.responseWorker)
        response_thread.start()
        # block till drone is connected
        while not self.connected:
            self.connected = self.command('command', timeout=1) 
            time.sleep(1)
        self.connected = True
        rospy.loginfo("connected, battery: %s" % self.command_str('battery?'))
        #rospy.loginfo("set speed: %s" % self.command_str('speed 20'))

        state_thread = Thread(target = self.stateWorker)
        state_thread.start()

        return self.connected

    # thread worder receiving responses to commands
    # if response received sets event
    def responseWorker(self):
        print ("Listener is waiting for responses...")
        while self.running:
            try:
                data, server = self.sock.recvfrom(1518)
                response = data.decode(encoding="utf-8")
                #print(response)
                self.response_queue.append(response.strip())
                self.response_received.set()
            except Exception as e:
                print('\nError %s' % e)
                #break
    
    # registers callback for processing the state
    def registerStateHandler(self, callback):
        self.state_handlers.append(callback)

    # thread worder for processing state messages
    # calls registered callbacks every time a state message arrives
    def stateWorker(self):
        rospy.loginfo("listening for state on port %d" % self.state_port)
        while self.running:
            try:
                data, server = self.state_recv_socket.recvfrom(1518)
                state = State(data.decode(encoding="utf-8"))
                for callback in self.state_handlers:
                    if callback is not None:
                        callback(state=state)
            except Exception as e:                
                rospy.loginfo('\nError %s' % e)
                break


    # blocking method to send command and return with the string result
    # wait for response by pulling event
    # TODO progress callback
    def command_str(self, cmd, timeout=30) -> str:
        if not timeout:
            timeout = self._command_timeout
        result = None
        self.send_command(cmd)
        self.response_received.wait(timeout=timeout)
        if self.response_queue:
            result = self.response_queue.pop()
            cmd = self.command_queue.pop()
            print('\n%s -> %s' % (cmd, result))
        else:
            print ('\n timeout')
        print(self.command_queue)
        return result


    # blocking method sends command and returns True if success
    def command(self, cmd, timeout=30) -> bool:
        return True if self.command_str(cmd, timeout) == 'ok' else False

    ## blocking method to send command and return with boolen success
    ## wait for response using thread.select
    ## deprecated: use command() instead
    def command_sync(self, cmd, timeout=10) -> bool:
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
    ## deprecated: use command_str() instead
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
        self.command_queue.append(cmd)
        self.response_received.clear()
        self.last_command_ts = time.time()
        return sent
    
    ## execute multiple commands after each other
    def execute_commands(self, cmds, sleep=0.1):
        for cmd in cmds:
            self.command(cmd)
            time.sleep(sleep)
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

    def keep_alive(self):
        #print('ka: %d' % (time.time() - self.last_command_ts))
        if (time.time() - self.last_command_ts) > 8:
            self.command_str('time?')

    # TODO: test
    def terminate(self):
        #self.response_thread.join()
        #self.state_thread.join()
        self.running = False
        self.background_frame_read.stop()


class State:

    state_dict = {}


    # Conversion functions for state protocol fields
    INT_STATE_FIELDS = (
        # Tello EDU with mission pads enabled only
        'mid', 'x', 'y', 'z',
        # 'mpry': (custom format 'x,y,z')
        # Common entries
        'pitch', 'roll', 'yaw',
        'vgx', 'vgy', 'vgz',
        'templ', 'temph',
        'tof', 'h', 'bat', 'time'
    )
    FLOAT_STATE_FIELDS = ('baro', 'agx', 'agy', 'agz')

    state_field_converters: Dict[str, Union[Type[int], Type[float]]]
    state_field_converters = {key : int for key in INT_STATE_FIELDS}
    state_field_converters.update({key : float for key in FLOAT_STATE_FIELDS})
        

    def __init__(self, raw=''):
        self.state_dict = self.parse(raw)

    def get(self, key):
        return self.state_dict.get(key)

    def parse(self, state: str) -> Dict[str, Union[int, float, str]]:
        """Parse a state line to a dictionary
        Internal method, you normally wouldn't call this yourself.
        """
        state = state.strip()
        #print('Raw state data: {}'.format(state))

        if state == 'ok':
            return {}

        self.state_dict = {}
        for field in state.split(';'):
            split = field.split(':')
            if len(split) < 2:
                continue

            key = split[0]
            value: Union[int, float, str] = split[1]

            if key in self.state_field_converters:
                num_type = self.state_field_converters[key]
                try:
                    value = num_type(value)
                except ValueError as e:
                    print('Error parsing state value for {}: {} to {}'
                                       .format(key, value, num_type))
                    print(e)
                    continue

            self.state_dict[key] = value

        return self.state_dict
    
    def __str__(self):
        return str(self.state_dict)

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

def main():
    print('\r\n\r\nTello Control.\r\n')
    print('type quit to quit.\r\n')

    tello = Tello()
    tello.connect()

    while True: 
        try:
            print ('$', end='', flush=True)
            msg = input("")
            if not msg:
                break
            if 'quit' in msg:
                tello.end()
                break
            print(tello.command_str(msg))

        except KeyboardInterrupt:
            print ('\n . . .\n')
            tello.end()
            break

if __name__ == '__main__':
    main()
