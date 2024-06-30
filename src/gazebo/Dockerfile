# Use the ROS Noetic image as the base
FROM ros:noetic-ros-base

# Create a workspace directory for the ROS project
WORKDIR /catkin_ws/src

#edit these for custom nodes
ENV PACKAGE_NAME gazebo
ENV LAUNCH_FILE gazebo_drone.launch
# Create a new ROS package for the package
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_create_pkg $PACKAGE_NAME rospy std_msgs"

# Copy the src of package into the container
COPY . /catkin_ws/src/$PACKAGE_NAME/

RUN apt-get update && apt-get install -y dos2unix python3 ros-noetic-gazebo-ros ros-noetic-robot-state-publisher ros-noetic-gazebo-ros-control

# Convert line endings in the Python script
RUN dos2unix /catkin_ws/src/$PACKAGE_NAME/src/*

# Change back to the root of the workspace
WORKDIR /catkin_ws

# Run catkin_make to build the ROS project
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/noetic/setup.bash  && catkin_make && source /catkin_ws/devel/setup.bash

# Set up environment variables for ROS
ENV ROS_DISTRO noetic
ENV ROS_WORKSPACE /catkin_ws

# Setup-Skript kopieren
RUN cp $ROS_WORKSPACE/devel/setup.sh $ROS_WORKSPACE/devel/share/gazebo/

# Source the ROS setup file every time a new container is run
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/$ROS_DISTRO/setup.bash && source /catkin_ws/devel/setup.bash && roslaunch $PACKAGE_NAME $LAUNCH_FILE --wait"]
