FROM osrf/ros:humble-desktop-full

COPY . /build
RUN /build/scripts/install/configure-workspace.bash
RUN /build/scripts/install/update-registries.bash
RUN rm -rf /build