# syntax=docker/dockerfile:1

# We use ROS2 Humble (https://docs.ros.org/en/humble/index.html) as base image
# In the future -base instead of -core might be necessary
FROM ros:humble-ros-core

# Install runtime dependencies
RUN apt update -q && apt install -qy zsh ros-$ROS_DISTRO-example-interfaces

# Ensure sourced ROS environment at startup
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
