#!/usr/bin/env python3


import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header, Bool
import os

def publish_occupancy_grid(grid, resolution, publisher):
    # Initialize the message
    grid_msg = OccupancyGrid()
    grid_msg.header = Header()
    grid_msg.header.stamp = rospy.Time.now()
    grid_msg.header.frame_id = "map"

    # Set the grid metadata (adjust according to your grid's configuration)
    grid_msg.info.resolution = resolution  # Grid resolution in meters/cell
    grid_msg.info.width = grid.shape[1]
    grid_msg.info.height = grid.shape[0]
    grid_msg.info.origin.position.x = 0.0
    grid_msg.info.origin.position.y = 0.0
    grid_msg.info.origin.position.z = 0.0
    grid_msg.info.origin.orientation.x = 0.0
    grid_msg.info.origin.orientation.y = 0.0
    grid_msg.info.origin.orientation.z = 0.0
    grid_msg.info.origin.orientation.w = 1.0

    # Specify the path where the file should be saved inside the container
    # file_path = '/catkin_ws/src/mapping/occ_grid.npy'
    #
    # if print_grid:
    #     np.save(file_path, custom_grid.grid)
    #     print("SAVED")

    # Flatten the grid array and convert it to a list for the message
    grid_msg.data = list(grid.flatten())

    # Publish the message
    publisher.publish(grid_msg)


rospy.init_node("static_map")
grid_pub = rospy.Publisher("/mapping/map", OccupancyGrid, queue_size=10)
resolution = 2

if __name__ == "__main__":
    # Change the current working directory
    script_dir = '/catkin_ws/src/static_map/scripts'
    os.chdir(script_dir)
    occ_grid = np.load("new_occ_grid.npy")
    while not rospy.is_shutdown():
        publish_occupancy_grid(occ_grid, resolution, grid_pub)
        rospy.sleep(3)

