FROM osrf/ros:humble-desktop-full

COPY . /app
WORKDIR /app

RUN scripts/install/configure-workspace.bash
RUN scripts/install/update-registries.bash
RUN scripts/install/install-packages.bash
RUN bash scripts/install/build-packages.bash
    
CMD [ "bash", "scripts/containers/twig-entrypoint.bash" ]
