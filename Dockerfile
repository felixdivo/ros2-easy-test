# syntax=docker/dockerfile:1

# NOTE: Keep this file in sync with .github/workflows/*.yaml

# We use Foxy Fitzroy (https://docs.ros.org/en/rolling/Releases.html) as base image,
# since it is the oldest ROS2 distro that we support and we want to ensure
# that we do not use APIs that are not available in older distros.
FROM ros:foxy-ros-core

# Install runtime dependencies
RUN apt-get update -q && apt-get install -qy python3-pip python-is-python3 git zsh ros-$ROS_DISTRO-example-interfaces

# Need to have setuptools version 64+ for editable installs
RUN pip install --upgrade pip setuptools

# Ensure sourced ROS environment at startup
RUN echo 'source /opt/ros/$ROS_DISTRO/setup.bash' >> ~/.bashrc
