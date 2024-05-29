# Start from a ROS base image
FROM ros:noetic-ros-base

# Install RViz
RUN apt-get update && apt-get install -y \
    ros-noetic-rviz \
    && rm -rf /var/lib/apt/lists/*

# Set the environment variable for the X11 display
ENV DISPLAY=host.docker.internal:0
ENV LIBGL_ALWAYS_SOFTWARE=1

# Run RViz as the default command
CMD ["rviz"]

