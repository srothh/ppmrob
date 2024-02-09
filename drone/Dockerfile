# Use the ROS Noetic image as the base
FROM ros:noetic-perception

# Create a workspace directory for the ROS project
#WORKDIR /catkin_ws/src

#edit these for custom nodes
ENV PACKAGE_NAME drone
ENV LAUNCH_FILE drone.launch

# Copy the src of package into the container
COPY . /catkin_ws/src/$PACKAGE_NAME

RUN apt-get update && apt-get install -y dos2unix python3 python3-pip netcat net-tools
RUN pip3 install av image

# Convert line endings in the Python script
RUN dos2unix /catkin_ws/src/$PACKAGE_NAME/action/*
RUN dos2unix /catkin_ws/src/$PACKAGE_NAME/src/*
RUN dos2unix /catkin_ws/src/$PACKAGE_NAME/src/action/*
RUN dos2unix /catkin_ws/src/$PACKAGE_NAME/src/tello/*
RUN dos2unix /catkin_ws/src/$PACKAGE_NAME/src/sensor/*
RUN dos2unix /catkin_ws/src/$PACKAGE_NAME/src/util/*

# Change back to the root of the workspace
WORKDIR /catkin_ws


# Set up environment variables for ROS
ENV ROS_DISTRO noetic
ENV ROS_WORKSPACE /catkin_ws

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash && source /catkin_ws/devel/setup.bash" >> /root/.bashrc
# Run catkin_make to build the ROS project
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/noetic/setup.bash && catkin_make && source /catkin_ws/devel/setup.bash

# Source the ROS setup file every time a new container is run
#ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/$ROS_DISTRO/setup.bash && source /catkin_ws/devel/setup.bash && roslaunch $PACKAGE_NAME $LAUNCH_FILE --wait"]
ENTRYPOINT ["/bin/bash", "-c", "while true; do sleep 1000; done"]
#ENTRYPOINT /root/${PACKAGE_NAME}/entrypoint.sh
 