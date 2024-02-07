#!/usr/bin/env python3

import rospy  # the library should be added as package dependency for the package on which working here
import drone.msg
import cv2

if __name__ == '__main__':
    try:
        rospy.init_node('safety_node')  # register the node with roscore, allowing it to communicate with other nodes
        # endless loop waiting to press any key
        # rospy.loginfo("Press any key to quit")
        client = actionlib.SimpleActionClient('emergency', drone.msg.EmergencyAction)
        client.wait_for_server()
        print("Press 'e' to send emergency command\nany other key to land")
        while True:
            k = cv2.waitKey(1) & 0xFF
            if k == ord('e'):
                client.send_goal_and_wait(drone.msg.EmergencyGoal(soft=False))
            else:  
                client.send_goal_and_wait(drone.msg.EmergencyGoal(soft=True))

    except rospy.ROSInterruptException:
        pass
