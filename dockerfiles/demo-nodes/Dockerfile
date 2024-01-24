# https://hub.docker.com/_/ros/
FROM ros:humble-ros-base

# install ros package
RUN apt-get update && apt-get install -y \
  ros-${ROS_DISTRO}-demo-nodes-cpp \
  ros-${ROS_DISTRO}-demo-nodes-py && \
  rm -rf /var/lib/apt/lists/*
