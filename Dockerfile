# syntax=docker/dockerfile:1

# NOTE: Keep this file in sync with .github/workflows/*.yaml

# We use ROS2 Humble (https://docs.ros.org/en/humble/index.html) as base image,
# since it is the oldest ROS2 distro that we support and we want to ensure
# that we do not use APIs that are not available in older distros.
FROM ros:humble-ros-core

# Install runtime dependencies
RUN apt-get update -q && apt-get install -qy git zsh ros-$ROS_DISTRO-example-interfaces

# Ensure sourced ROS environment at startup
RUN echo 'source /opt/ros/$ROS_DISTRO/setup.bash' >> ~/.bashrc
