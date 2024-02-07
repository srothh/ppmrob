# Use the ROS Noetic image as the base
FROM ros:noetic-ros-base

# Create a workspace directory for the ROS project
#WORKDIR /catkin_ws/src

#edit these for custom nodes
ENV PACKAGE_NAME safety
ENV LAUNCH_FILE safety.launch


# Copy the src of package into the container
COPY . /root/$PACKAGE_NAME/


RUN apt-get update && apt-get install -y dos2unix python3  python3-pip
RUN pip3 install keyboard

# Convert line endings in the Python script

# Change back to the root of the workspace
WORKDIR /catkin_ws


# Set up environment variables for ROS
ENV ROS_DISTRO noetic
ENV ROS_WORKSPACE /catkin_ws

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash && source /catkin_ws/devel/setup.bash" >> /root/.bashrc

# Source the ROS setup file every time a new container is run
#ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/$ROS_DISTRO/setup.bash && source /catkin_ws/devel/setup.bash && roslaunch $PACKAGE_NAME $LAUNCH_FILE --wait"]
#ENTRYPOINT ["/bin/bash", "-c", "while true; do sleep 1000; done"]
ENTRYPOINT /root/${PACKAGE_NAME}/entrypoint.sh
