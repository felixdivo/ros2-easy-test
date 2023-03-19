# syntax=docker/dockerfile:1

# Maybe we will need -base instead of -core at some point
FROM ros:humble-ros-core

RUN apt update -q && apt install -qy git zsh ros-$ROS_DISTRO-example-interfaces

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
