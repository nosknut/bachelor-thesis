# https://hub.docker.com/_/ros/
FROM ros:humble-ros-base

# install ros package
RUN apt-get update && apt-get install -y \
      ros-${ROS_DISTRO}-rviz2 \
      ros-${ROS_DISTRO}-rqt && \
    rm -rf /var/lib/apt/lists/*
