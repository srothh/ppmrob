#!/bin/bash

#mkdir -p /catkin_ws/src
cd /catkin_ws/src

cp -r /root/$PACKAGE_NAME .

source /opt/ros/noetic/setup.bash
catkin_create_pkg ${PACKAGE_NAME} rospy std_msgs actionlib_msgs actionlib message_generation

cd /catkin_ws
catkin_make

while true; do sleep 1000; done
