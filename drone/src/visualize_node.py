#!/usr/bin/env python3

import rospy  # the library should be added as package dependency for the package on which working here
import common.msg
import cv2
import actionlib
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt

positions = []



def odometry_callback(data: PoseStamped):
    global positions
    coordinates = (data.pose.position.x, data.pose.position.y, data.pose.position.z)
    positions.append(coordinates)
    

if __name__ == '__main__':
    try:
        rospy.init_node('visualize_node')  # register the node with roscore, allowing it to communicate with other nodes
        battery_subscriber = rospy.Subscriber('/odometry/return_signal', PoseStamped, callback=odometry_callback)
        
        while not rospy.is_shutdown():
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                "key_release_event",
                lambda event: [exit(0) if event.key == "escape" else None],
            )
            if positions:
                x, y, z = zip(*positions)
                plt.plot(y, x, "-ob")
            #plt.plot(px, py, "-r")
            #plt.plot(ipx, ipy, "or")
            plt.axis("equal")
            plt.grid(True)
            plt.pause(1.0)    
#        rospy.spin()
    except rospy.ROSInterruptException:
        pass
