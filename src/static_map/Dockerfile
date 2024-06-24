# Use the ROS Noetic image as the base
FROM ros:noetic-ros-base

# Create a workspace directory for the ROS project
WORKDIR /catkin_ws/src

#edit these for custom nodes
ENV PACKAGE_NAME static_map
ENV LAUNCH_FILE static_map.launch
# Create a new ROS package for the listener
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_create_pkg $PACKAGE_NAME std_msgs rospy geometry_msgs nav_msgs"

RUN apt-get update && apt-get install -y dos2unix
RUN apt-get update && apt-get install -y python3
RUN apt-get install -y python3-pip
# RUN apt-get update && apt-get install ffmpeg libsm6 libxext6  -y

# Copy the static_map package into the container
COPY src/$PACKAGE_NAME/ /catkin_ws/src/$PACKAGE_NAME/

# Convert line endings in the Python script
RUN dos2unix /catkin_ws/src/$PACKAGE_NAME/scripts/*

# Change back to the root of the workspace
WORKDIR /catkin_ws

# Run catkin_make to build the ROS project
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/noetic/setup.bash  && catkin_make && source /catkin_ws/devel/setup.bash

# Set up environment variables for ROS
ENV ROS_DISTRO noetic
ENV ROS_WORKSPACE /catkin_ws

RUN chmod +x /catkin_ws/src/${PACKAGE_NAME}/scripts/*
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Source the ROS setup file every time a new container is run
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/$ROS_DISTRO/setup.bash && source /catkin_ws/devel/setup.bash && roslaunch $PACKAGE_NAME $LAUNCH_FILE --wait"]
