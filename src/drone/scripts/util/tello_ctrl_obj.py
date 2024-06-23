# tello SDK boiler plate
# usig tello library

from tello import Tello

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
