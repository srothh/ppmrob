# Use the ROS Noetic image as the base
FROM ros:noetic-ros-base

# Create a workspace directory for the ROS project
WORKDIR /catkin_ws/src

# Create a new ROS package for the listener
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_create_pkg publisher rospy std_msgs"

# Copy the listener package into the container
COPY . /catkin_ws/src/publisher/

RUN apt-get update && apt-get install -y dos2unix
RUN apt-get update && apt-get install -y python3

# Convert line endings in the Python script
RUN dos2unix /catkin_ws/src/publisher/src/publisher_node.py

# Change back to the root of the workspace
WORKDIR /catkin_ws

# Run catkin_make to build the ROS project
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/noetic/setup.bash  && catkin_make && source /catkin_ws/devel/setup.bash

# Set up environment variables for ROS
ENV ROS_DISTRO noetic
ENV ROS_WORKSPACE /catkin_ws
RUN chmod +x /catkin_ws/src/publisher/src/publisher_node.py
# Source the ROS setup file every time a new container is run
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/$ROS_DISTRO/setup.bash && source /catkin_ws/devel/setup.bash && roslaunch publisher publisher.launch"]
