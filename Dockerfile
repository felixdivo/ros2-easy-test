# syntax=docker/dockerfile:1

# Maybe we will need -build at some point
FROM ros:humble-ros-core

RUN apt update && apt install -y git zsh

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
