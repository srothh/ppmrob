#! /usr/bin/env python3

from __future__ import print_function

import rospy
# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import tello.msg
from drone_action_client import DroneActionClient
import time

    
def test_move(client):
    client.takeoff()
    client.rotate(45)
    client.rotate(-45)
    client.move_x(50)
    client.move_x(-50)
    client.move_y(50)
    client.move_y(-50)
    client.move_z(-50)
    client.move_z(50)
    time.sleep(15)
    client.land()

def test_grid_2by2(client):
    client.takeoff()
    client.move_x(200)
    client.rotate(90)
    client.move_x(50)
    client.rotate(90)
    client.move_x(200)
    client.rotate(-90)
    client.move_x(50)
    client.rotate(-90)
    client.move_x(200)
    client.rotate(90)
    client.move_x(50)
    client.rotate(90)
    client.move_x(200)
    client.rotate(90)
    client.move_x(200)
    client.land()


if __name__ == '__main__':
    try:
        rospy.init_node('launch_client_callbacks_py')
        # init drone client
        client = DroneActionClient()

        test_move(client)
        #test_grid_2by2(client) 

        rospy.spin()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
