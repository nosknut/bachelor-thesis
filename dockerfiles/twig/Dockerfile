FROM osrf/ros:humble-desktop-full

COPY . /root/ros2_ws
WORKDIR /root/ros2_ws

RUN bash scripts/setup/configure-workspace.bash
RUN bash scripts/ros2/install-dependencies.bash
RUN bash scripts/ros2/build-packages.bash
    
CMD [ "bash", "/root/ros2_ws/dockerfiles/twig/entrypoint.bash" ]
