# Use the ROS Noetic image as the base
FROM ros:noetic-ros-base

# Set up environment variables for ROS
ENV ROS_DISTRO noetic
ENV ROS_WORKSPACE /catkin_ws

# Create a workspace directory for the ROS project
WORKDIR ${ROS_WORKSPACE}/src

#edit these for custom nodes
ENV PACKAGE_NAME keyboard
# Create a new ROS package for the package
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_create_pkg $PACKAGE_NAME roscpp std_msgs"

# Copy the src of package into the container
COPY . ${ROS_WORKSPACE}/src/$PACKAGE_NAME/

# Change back to the root of the workspace
WORKDIR /catkin_ws

# Install SDL
RUN apt-get update && apt-get install -y libsdl1.2-dev

# Run catkin_make to build the ROS project
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/noetic/setup.bash  && catkin_make && source /catkin_ws/devel/setup.bash

# Set the TERM environment variable
ENV TERM xterm
# Source the ROS setup file every time a new container is run
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/$ROS_DISTRO/setup.bash && source $ROS_WORKSPACE/devel/setup.bash && rosrun $PACKAGE_NAME keyboard"]