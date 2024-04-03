#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Polygon
import numpy as np
import cv2


# Assumes square window and absolute drone position
def map_coordinate(x, y, x_d, y_d, window_size):
    x_p = int(x_d) - window_size//2 + x
    y_p = int(y_d) - window_size//2 + y
    return x_p, y_p

class OccupancyGrid:
    def __init__(self, width, height, resolution):
        self.height = height
        self.width = width
        self.grid = np.full((height, width), -1)  # -1 for unexplored, 0 for free, 1 for occupied
        self.mask = np.full((height, width), 0)
        self.resolution = resolution

    def update_cell(self, x, y, value):
        # Ensure coordinates are within the grid bounds
        if 0 <= x < self.grid.shape[1] and 0 <= y < self.grid.shape[0]:
            self.grid[y, x] = value

    def world_to_grid(self, world_x, world_y):
        return int(world_x / self.resolution), int(world_y / self.resolution)

    def resize(self, new_width, new_height):
        new_width = max(new_width, self.width)
        new_height = max(new_height, self.height)
        if new_width <= self.width and new_height <= self.height:
            return

        new_grid = np.full((new_height, new_width), -1)
        new_mask = np.full((new_height, new_width), 0)
        new_grid[0:self.grid.shape[0], 0:self.grid.shape[1]] = self.grid
        new_mask[0:self.mask.shape[0], 0:self.mask.shape[1]] = self.mask
        self.grid = new_grid
        self.mask = new_mask
        self.height = new_height
        self.width = new_width

    def update_occupancy_grid(self, drone_pos, lines, victims):
        x_d = drone_pos[0]
        y_d = drone_pos[1]
        if lines is not None:
            for line in lines:
                # Convert world coordinates to grid coordinates
                x1, y1, x2, y2 = line[0]
                p1 = map_coordinate(x1, y1, x_d, y_d, 100)
                p2 = map_coordinate(x2, y2, x_d, y_d, 100)
                grid_x1, grid_y1 = self.world_to_grid(p1[0], p1[1])
                grid_x2, grid_y2 = self.world_to_grid(p2[0], p2[1])

                if grid_x1 > self.width or grid_x2 > self.width or grid_y1 > self.height or grid_y2 > self.height:
                    new_grid_width = max(2*self.width, max(grid_x1, grid_x2))
                    new_grid_height = max(2*self.height, max(grid_y1, grid_y2))
                    self.resize(new_grid_width, new_grid_height)

                # Update the grid cells along the line as occupied
                cv2.line(self.mask, (grid_x1, grid_y1), (grid_x2, grid_y2), 1, 1)  # Mark as occupied

                # Find the indices where the grid is unexplored (-1) and the mask has the line drawn (1)
                update_indices = np.where((self.grid == -1) & (self.mask == 1))

                self.grid[update_indices] = 1

        if victims is not None:
            for victim in victims:
                upper_left = victim[0]
                bottom_right = victim[1]
                x_c = (upper_left[0] + bottom_right[0])/2
                y_c = (upper_left[1] + bottom_right[1])/2

                p = map_coordinate(x_c, y_c, x_d, y_d, 100)
                victim_x, victim_y = self.world_to_grid(p[0], p[1])

                cv2.circle(self.grid, (victim_x, victim_y), 2, 1, 5)



def line_callback(polygon: Polygon):
    lines = []
    drone_pos = None # TODO: Get drone position from odometry
    for i in range(0, len(polygon), 2):
        p1 = polygon.points[i]
        p2 = polygon.points[i + 1]
        x1, y1, x2, y2 = p1[0], p1[1], p2[0], p2[1]


grid = OccupancyGrid(250, 250, 1)



if __name__ == '__main__':
    rospy.init_node('mapping')