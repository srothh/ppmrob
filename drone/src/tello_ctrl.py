# tello SDK boiler plate

import threading 
import socket
import sys
import time

HOST = ''
PORT = 9000

def recv():
    global sock
    count = 0
    while True: 
        try:
            print('receiving')
            data, server = sock.recvfrom(1518)
            print("UDP ",count," from ", server, ": ", data.decode(encoding="utf-8"))
            count += 1
        except Exception:
            print ('\nExit . . .\n')
            break


def main():
    print('\r\n\r\nTello Control.\r\n')
    print('type end to quit.\r\n')
    
    # Create a UDP socket
    local_addr = (HOST,PORT) 
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tello_address = ('192.168.10.1', 8889)
    sock.bind(local_addr)

    #create recv thread
    recv_thread = threading.Thread(target=recv)
    recv_thread.start()

    while True: 
        try:
            msg = input("")
            if not msg:
                break
            if 'end' in msg:
                print ('closing...')
                sock.close()  
                break

            # Send data
            msg = msg.encode(encoding="utf-8") 
            sent = sock.sendto(msg, tello_address)
            print("sent: ",sent)
        except KeyboardInterrupt:
            print ('\n . . .\n')
            sock.close()  
            break


if __name__ == '__main__':
    main()
