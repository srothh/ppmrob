#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Polygon

class Map:
    def __init__(self):
        self.width = 100
        self.height = 100

    def update_map_size(self, width, height):
        self.width = width
        self.height = height
        # TODO: Think about how to keep all the recent information!


def line_callback(polygon):
    p1 = polygon.points[0]
    p2 = polygon.points[1]



if __name__ == '__main__':
    rospy.init_node('mapping')