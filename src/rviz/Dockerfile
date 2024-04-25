# Start from a ROS base image
FROM ros:noetic-ros-base

# Install RViz
RUN apt-get update && apt-get install -y \
    ros-noetic-rviz \
    && rm -rf /var/lib/apt/lists/*

# Set the environment variable for the X11 display
ENV DISPLAY=host.docker.internal:0

# Run RViz as the default command
#CMD ["rviz"]
ENTRYPOINT ["/bin/bash", "-c", "while true; do sleep 1000; done"]

