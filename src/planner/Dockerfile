# Use the custom ROS image with py_trees dependencies as the base
FROM petervrabel/my-py-trees-ros-img:latest

# Create a workspace directory for the ROS project
WORKDIR /catkin_ws/src

#edit these for custom nodes
ENV PACKAGE_NAME planner
ENV LAUNCH_FILE planner.launch
# Create a new ROS package for the package
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash"



RUN apt-get update && apt-get install -y \
    dos2unix \
    python3

# Copy the src of package into the container
COPY src/$PACKAGE_NAME/ /catkin_ws/src/$PACKAGE_NAME/
COPY src/common/ /catkin_ws/src/common/
COPY src/control/action/ /catkin_ws/src/control/action/
COPY src/control/CMakeLists.txt src/control/package.xml /catkin_ws/src/control/

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

# Give execute permissions to the Python script
RUN chmod +x /catkin_ws/src/${PACKAGE_NAME}/scripts/*

# Source the ROS setup file every time a new container is run
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/$ROS_DISTRO/setup.bash && source /catkin_ws/devel/setup.bash && roslaunch $PACKAGE_NAME $LAUNCH_FILE number_of_victims:=$NUMBER_OF_VICTIMS interactive:=$INTERACTIVE_WAYPOINTS waypoints:=$WAYPOINTS --wait"]
# `--wait` waits for master to start before launching