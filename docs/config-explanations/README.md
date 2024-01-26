[<-- Back](/README.md)

# Config Explanations

## GUI in Docker Containers
Based on [this post](https://stackoverflow.com/questions/73092750/how-to-show-gui-apps-from-docker-desktop-container-on-windows-11), use the following configuration to pass the WSL display to the container.

```yml
version: "3"
services:
  gui-service:
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix
      - /mnt/wslg:/mnt/wslg
    environment:
      - DISPLAY
      - WAYLAND_DISPLAY
      - XDG_RUNTIME_DIR
      - PULSE_SERVER
```

When this is done, run ```docker-compose up gui-service``` from the WSL terminal.

## Extending Docker Compose Service

Other services can [extend](https://docs.docker.com/compose/compose-file/05-services/#extends) the gui-service so the configuration does not need to be repeated.

```yml
version: "3"
services:
  some-service:
    extends:
      service: gui-service
    command: echo "Hello World!"
```

When this is done, run ```docker-compose up some-service``` from the WSL terminal.

## Shared Memory

As explained by [this](https://answers.ros.org/question/370595/ros2-foxy-nodes-cant-communicate-through-docker-container-border/) post When ROS2 Discovery detects another node on the same network, it will try to use shared memory to communicate. This is faster than using the network. However, if the nodes are running in different containers, they will not be able to use shared memory. To fix this, the following configuration from [this](https://answers.ros.org/question/370595/ros2-foxy-nodes-cant-communicate-through-docker-container-border/) post can be used.

```yml
version: "3"
services:
  some-service:
    volumes:
      - /dev/shm/:/dev/shm
```