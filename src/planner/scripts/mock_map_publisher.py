#!/usr/bin/env python3
import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header

import common.config.defaults as defaults  # TODO add to dockerfile as per issue!

def publish_occupancy_grid():
    rospy.init_node('mock_occupancy_grid_publisher', anonymous=True)
    pub = rospy.Publisher(defaults.Mapping.OCCUPANCY_GRID_TOPIC_NAME, OccupancyGrid, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        # Create a dummy occupancy grid
        grid = OccupancyGrid()
        grid.header = Header(stamp=rospy.Time.now(), frame_id="world")
        grid.info.resolution = 1.0  # 1 meter per cell
        grid.info.width = 10  # 10 cells wide
        grid.info.height = 10  # 10 cells tall
        grid.info.origin.position.x = 0.0
        grid.info.origin.position.y = 0.0
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.x = 0.0
        grid.info.origin.orientation.y = 0.0
        grid.info.origin.orientation.z = 0.0
        grid.info.origin.orientation.w = 1.0
        # Fill the grid with zeros (free space)
        # grid.data = [0] * 100  # 10x10 grid
        grid.data = [
            100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
            100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
            100, 100,   0,   0,   0,   0,   0,   0,   0, 100,
            100, 100,   0, 100, 100, 100, 100,   1, 100, 100,
            100, 100,   0, 100, 100, 100, 100, 100, 100, 100,
            100, 100,   0, 100, 100, 100, 100, 100, 100, 100,
            100, 100,   0, 100, 100, 100, 100, 100, 100, 100,
            100, 100,   0, 100, 100, 100, 100, 100, 100, 100,
            100, 100,   0, 100, 100, 100, 100, 100, 100, 100,
            100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
        ]


        # Publish the occupancy grid
        pub.publish(grid)
        rospy.loginfo("Published mock occupancy grid")
        rate.sleep()
if __name__ == '__main__':
    try:
        publish_occupancy_grid()
    except rospy.ROSInterruptException:
        pass
