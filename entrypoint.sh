#!/bin/bash

# copy sources to the shared src volume
cd /catkin_ws/src

cp -r /root/$PACKAGE_NAME .

#convert dos endlines
find /catkin_ws/src/$PACKAGE_NAME -type f -print0 | xargs -0 dos2unix -

source /opt/ros/noetic/setup.bash
catkin_create_pkg ${PACKAGE_NAME} rospy std_msgs actionlib_msgs actionlib message_generation

cd /catkin_ws
catkin_make

while true; do sleep 1000; done
