#!/usr/bin/env python3

import rospy 
import actionlib  # the library should be added as package dependency for the package on which working here
import drone.msg
import keyboard


if __name__ == '__main__':
    try:
        rospy.init_node('safety_node')  # register the node with roscore, allowing it to communicate with other nodes
        # endless loop waiting to press any key
        # rospy.loginfo("Press any key to quit")
        client = actionlib.SimpleActionClient('emergency', drone.msg.EmergencyAction)
        client.wait_for_server()
        while True:
            print('Press \'e\' to send emergency command\nany other key to land')
            event = keyboard.read_event()
            if event.event_type == keyboard.KEY_DOWN and event.name == 'e':
                client.send_goal_and_wait(drone.msg.EmergencyGoal(soft=False))
            else:  
                client.send_goal_and_wait(drone.msg.EmergencyGoal(soft=True))

    except rospy.ROSInterruptException:
        pass
