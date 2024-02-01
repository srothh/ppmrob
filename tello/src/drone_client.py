#! /usr/bin/env python3

import time
from djitellopy import Tello


tello = Tello()

tello.connect()
tello.takeoff()

print(tello.get_battery())
tello.set_speed(10)
time.sleep(1)


print('forward')
tello.send_command_without_return('forward 200')

print('sleep')
time.sleep(2)
if  tello.get_own_udp_object()['responses']: print('response: %s' % tello.get_own_udp_object()['responses'].pop(0))

print('stop')
tello.send_command_without_return('stop')
time.sleep(1)
if  tello.get_own_udp_object()['responses']: print('response: %s' % tello.get_own_udp_object()['responses'].pop(0))

print('back')
tello.send_command_with_return('back 100')

time.sleep(15)
tello.land()


