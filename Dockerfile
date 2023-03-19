# syntax=docker/dockerfile:1

# Maybe we will need -build at some point
FROM ros:humble-ros-core

RUN apt update && apt install -y git zsh ros-$ROS_DISTRO-demo-nodes-cpp

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
