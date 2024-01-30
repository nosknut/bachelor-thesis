FROM ghcr.io/ros-planning/moveit2_tutorials:humble-tutorial

# https://github.com/ros2/rviz/issues/948#issuecomment-1427569107
RUN apt update -y
RUN apt upgrade -y
RUN apt install software-properties-common -y
RUN add-apt-repository ppa:kisak/kisak-mesa
RUN apt update -y
RUN apt upgrade -y