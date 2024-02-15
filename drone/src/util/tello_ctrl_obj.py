# tello SDK boiler plate

import threading 
import socket
import sys
import time
import select
from tello import Tello

def main():
    print('\r\n\r\nTello Control.\r\n')
    print('type quit to quit.\r\n')

    tello = Tello()

    while True: 
        try:
            print ('$', end='', flush=True)
            msg = input("")
            if not msg:
                break
            if 'quit' in msg:
                tello.end()
                break
            tello.command(msg)

        except KeyboardInterrupt:
            print ('\n . . .\n')
            tello.end()
            break

if __name__ == '__main__':
    main()
