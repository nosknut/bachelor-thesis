# https://hub.docker.com/_/ros/
FROM osrf/ros:humble-desktop-full

RUN sudo apt-get update
RUN rosdep update --rosdistro=$ROS_DISTRO

RUN echo "\n\
if [ -f /workspace/install/local_setup.bash ]; then\n\
    source \"/workspace/install/local_setup.bash\"\n\
else\n\
    source \"/opt/ros/${ROS_DISTRO}/setup.bash\"\n\
fi\n\
" >> ~/.bashrc
