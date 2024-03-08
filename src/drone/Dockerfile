# Use the ROS Noetic image as the base
FROM ros:noetic-perception

# Create a workspace directory for the ROS project
WORKDIR /catkin_ws/src

#edit these for custom nodes
ENV PACKAGE_NAME drone
ENV LAUNCH_FILE drone.launch
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_create_pkg $PACKAGE_NAME rospy std_msgs"

# Copy the src of package into the container
COPY ./$PACKAGE_NAME/ /catkin_ws/src/$PACKAGE_NAME/
COPY ./common/ /catkin_ws/src/common/
#COPY ./control/ /catkin_ws/src/control/

#RUN add-apt-repository ppa:deadsnakes/ppa
RUN apt-get update && apt-get install -y dos2unix python3.10 python3-pip netcat net-tools python3-tk
RUN pip3 install av image matplotlib pyqt5

# Convert line endings in the Python script
#RUN find /catkin_ws/src/$PACKAGE_NAME -type f -print0 | xargs -0 dos2unix -

# Change back to the root of the workspace
WORKDIR /catkin_ws


# Run catkin_make to build the ROS project
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/noetic/setup.bash  && catkin_make && source /catkin_ws/devel/setup.bash


# Set up environment variables for ROS
ENV ROS_DISTRO noetic
ENV ROS_WORKSPACE /catkin_ws

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash && source /catkin_ws/devel/setup.bash" >> /root/.bashrc

# Source the ROS setup file every time a new container is run
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/$ROS_DISTRO/setup.bash && source /catkin_ws/devel/setup.bash && roslaunch $PACKAGE_NAME $LAUNCH_FILE --wait"]
#ENTRYPOINT ["/bin/bash", "-c", "while true; do sleep 1000; done"]
#ENTRYPOINT /root/${PACKAGE_NAME}/entrypoint.sh
 