#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PolygonStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
import numpy as np
import quaternion # Still need to install
import cv2
from collections import deque
import math

fov_x = 200
fov_y = 200

class CircularBuffer:
    def __init__(self, capacity):
        self.buffer = deque(maxlen=capacity)

    def append(self, item):
        self.buffer.append(item)

    def get_buffer(self):
        return list(self.buffer)

max_msgs = 1000
odometry_msgs = CircularBuffer(max_msgs)

def calculate_fov_size(diagonal_fov_degrees, height):
    # Camera resolution aspect ratio: 162:121
    aspect_ratio_width = 162
    aspect_ratio_height = 121

    # Convert diagonal FoV from degrees to radians
    diagonal_fov_radians = math.radians(diagonal_fov_degrees)

    # Calculate the horizontal and vertical FoV
    horizontal_fov_radians = 2 * math.atan(math.tan(diagonal_fov_radians / 2) * (aspect_ratio_width / math.sqrt(aspect_ratio_width**2 + aspect_ratio_height**2)))
    vertical_fov_radians = 2 * math.atan(math.tan(diagonal_fov_radians / 2) * (aspect_ratio_height / math.sqrt(aspect_ratio_width**2 + aspect_ratio_height**2)))

    # Calculate the field of view dimensions at the given height
    width = 2 * height * math.tan(horizontal_fov_radians / 2)
    depth = 2 * height * math.tan(vertical_fov_radians / 2)

    return width, depth


# Assumes square window and absolute drone position
def map_coordinate(x, y, x_d, y_d, fov_width, fov_height):
    x_p = int(x_d) - fov_width//2 + x
    y_p = int(y_d) - fov_height//2 + y
    return x_p, y_p

def transform_ros_point(point_msg, orientation_quaternion):
    # Extract point data from the message
    point = np.array([point_msg.x, point_msg.y, point_msg.z])

    # Convert the point to a pure quaternion (zero real part)
    point_quaternion = np.quaternion(0, *point)

    # Apply the transformation: p' = q^(-1) * p * q
    transformed_point_quaternion = orientation_quaternion.inverse() * point_quaternion * orientation_quaternion

    # Extract the vector part
    transformed_point = quaternion.as_float_array(transformed_point_quaternion)[1:]

    return transformed_point

class CustomOccupancyGrid:
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

    def update_fov(self, drone_pos, fov_size):
        # Convert the drone's position to grid coordinates
        drone_grid_x, drone_grid_y = self.world_to_grid(*drone_pos)
        buffer_x = int(fov_size[0]*0.05)
        buffer_y = int(fov_size[1]*0.05)

        # Calculate the top-left and bottom-right corners of the FOV in grid coordinates
        half_fov_x, half_fov_y = self.world_to_grid(fov_size[0] // 2 - buffer_x, fov_size[1] // 2 - buffer_y)
        top_left_x = max(drone_grid_x - half_fov_x, 0)
        top_left_y = max(drone_grid_y - half_fov_y, 0)
        bottom_right_x = min(drone_grid_x + half_fov_x, self.grid.shape[1] - 1)
        bottom_right_y = min(drone_grid_y + half_fov_y, self.grid.shape[0] - 1)

        # print(top_left_x, top_left_y, bottom_right_x, bottom_right_y)

        # Update the grid cells within the FOV to mark them as free
        # Ensure to check bounds to avoid out-of-index errors
        self.grid[top_left_y - 1:bottom_right_y, top_left_x - 1:bottom_right_x] = np.where(self.grid[top_left_y - 1:bottom_right_y, top_left_x - 1:bottom_right_x] != 100, 0, 100)

    def update_lines(self, drone_pos, lines, fov_width, fov_height):
        x_d = drone_pos[0]
        y_d = drone_pos[1]
        if lines is not None:
            for line in lines:
                # Convert world coordinates to grid coordinates
                x1, y1, x2, y2 = line[0]
                p1 = map_coordinate(x1, y1, x_d, y_d, fov_width, fov_height)
                p2 = map_coordinate(x2, y2, x_d, y_d, fov_width, fov_height)
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

                self.grid[update_indices] = 100

def lines_callback(data: PolygonStamped):
    num_lines = len(data.polygon)
    timestamp = data.header.stamp.to_sec()
    current_positions = odometry_msgs.get_buffer
    closest_msg = current_positions[-1]
    closest_diff = abs(closest_msg[1] - timestamp)
    for msg in current_positions[::-1]:
        if abs(msg[1] - timestamp) < closest_diff:
            closest_msg = msg
            closest_diff = msg[1] - timestamp
        if abs(msg[1] - timestamp) > closest_diff:
            break
    lines = []
    pos_point = closest_msg[0].position
    drone_pos = pos_point.x + 1000, pos_point.y + 1000 # Avoid negative values for now
    for i in range(0, num_lines, 2):
        p1 = transform_ros_point(data.polygon.points[i])
        p2 = transform_ros_point(data.polygon.points[i + 1])
        x1, y1, x2, y2 = p1[0], p1[1], p2[0], p2[1]
        lines.append((x1, y1, x2, y2))
    
    occ_grid.update_lines(drone_pos, lines, fov_x, fov_y)

def victim_callback(data: PolygonStamped):
    return

    

def odometry_callback(data: PoseStamped):
    global odometry_msgs
    odometry_msgs.append((data.pose, data.header.stamp.to_sec()))


def publish_occupancy_grid(custom_grid: CustomOccupancyGrid):
    # Initialize the message
    grid_msg = OccupancyGrid()
    grid_msg.header = Header()
    grid_msg.header.stamp = rospy.Time.now()
    grid_msg.header.frame_id = "world"  # or another appropriate frame

    # Set the grid metadata (adjust according to your grid's configuration)
    grid_msg.info.resolution = custom_grid.resolution  # Grid resolution in meters/cell
    grid_msg.info.width = custom_grid.grid.shape[1]
    grid_msg.info.height = custom_grid.grid.shape[0]
    grid_msg.info.origin.position.x = 0.0
    grid_msg.info.origin.position.y = 0.0
    grid_msg.info.origin.position.z = 0.0
    grid_msg.info.origin.orientation.x = 0.0
    grid_msg.info.origin.orientation.y = 0.0
    grid_msg.info.origin.orientation.z = 0.0
    grid_msg.info.origin.orientation.w = 1.0

    # Flatten the grid array and convert it to a list for the message
    grid_msg.data = list(custom_grid.grid.flatten())

    # Publish the message
    grid_pub.publish(grid_msg)
    


if __name__ == '__main__':
    rospy.init_node('mapping')
    occ_grid = CustomOccupancyGrid(250, 250, 0.01)
    odometry_subscriber = rospy.Subscriber('/odometry/return_signal', PoseStamped, callback=odometry_callback)
    lines_subsriber = rospy.Subscriber('/cv/lines', PolygonStamped, callback=lines_callback)
    grid_pub = rospy.Publisher('/mapping/occupancy_grid', OccupancyGrid, queue_size=10)
    step = 0
    while not rospy.is_shutdown:
        curr_time = rospy.Time().to_sec()
        if step % 5 == 0:
            current_positions = odometry_msgs.get_buffer
            closest_msg = current_positions[-1]
            pos_point = closest_msg[0].position
            drone_pos = pos_point.x, pos_point.y
            fov = calculate_fov_size(82.6, pos_point.z)
            fov_x, fov_y = fov[0], fov[1]
            occ_grid.update_fov(drone_pos, fov)
        step += 1
    
