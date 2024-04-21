#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Point32, PolygonStamped
import json
import os

# Initialize ROS node
rospy.init_node('simulation_data_publisher')

# Publishers
odom_pub = rospy.Publisher('/odometry/return_signal', PoseStamped, queue_size=10)
line_pub = rospy.Publisher('/cv/lines', PolygonStamped, queue_size=10)
victim_pub = rospy.Publisher('cv/victims', PolygonStamped, queue_size=10)

# Change the current working directory
script_dir = '/catkin_ws/src/mapping_simulation/scripts'
os.chdir(script_dir)

# Load simulation data
with open('sim_data_victims.json', 'r') as infile:
    simulation_data = json.load(infile)

# Helper function to create a PoseStamped message from position data
def create_pose_stamped_msg(x, y, time):
    msg = PoseStamped()
    msg.header.stamp = rospy.Time(time)
    msg.header.frame_id = "world"  # Or any other relevant frame
    msg.pose.position.x = x
    msg.pose.position.y = y
    # Assuming a flat 2D simulation for z and orientation
    msg.pose.position.z = 0
    msg.pose.orientation.w = 1.0
    return msg

# Helper function to create a PolygonStamped message from line data
def create_polygon_stamped_line_msg(lines, time):
    msg = PolygonStamped()
    msg.header.stamp = rospy.Time(time)
    msg.header.frame_id = "map"  # Or any other relevant frame
    for l in lines:
        line = l[0]
        # Line is assumed to be [x1, y1, x2, y2]
        start_point = Point32(x=line[0], y=line[1], z=0)  # z is ignored
        end_point = Point32(x=line[2], y=line[3], z=0)  # z is ignored
        msg.polygon.points.extend([start_point, end_point])
    return msg


def create_polygon_stamped_victim_msg(victims, time):
    msg = PolygonStamped()
    msg.header.stamp = rospy.Time(time)
    msg.header.frame_id = "map"  # Or any other relevant frame
    for victim in victims:
        victim_point = Point32(x=victim[0], y=victim[1], z=0)  # z is ignored
        msg.polygon.points.extend([victim_point, victim_point])  # TODO: Add imaginary bounding box for simulation
    return msg

# Publish data
rate = rospy.Rate(10)  # Adjust as needed
for timestep in simulation_data['timesteps']:
    keys = timestep.keys()
    # Publish odometry
    if 'drone_pos' in keys:
        od_msg = create_pose_stamped_msg(timestep['drone_pos']['x'], timestep['drone_pos']['y'], timestep['od_time'])
        odom_pub.publish(od_msg)
        # print(f"Published drone position {timestep['drone_pos']['x'], timestep['drone_pos']['y']}")

    # Publish lines
    if 'lines' in keys:
        line_msg = create_polygon_stamped_line_msg(timestep['lines'], timestep['line_time'])
        line_pub.publish(line_msg)
        # print(f"Published line {timestep['lines']}")

    # Publish lines
    if 'victims' in keys:
        victim_msg = create_polygon_stamped_victim_msg(timestep['victims'], timestep['victim_time'])
        victim_pub.publish(victim_msg)
        print(f"Published victim {timestep['victims']}")

    rate.sleep()

# Keep node alive until shut down
rospy.spin()
