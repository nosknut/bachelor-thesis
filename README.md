# Setup

## Windows (WSL)

- [WSL Setup](docs/setup/wsl/wsl/README.md)
- [Docker Setup](docs/setup/wsl/docker/README.md)

## Ubuntu

- [ROS2 Setup](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
- [Gazebo](https://gazebosim.org/docs/fortress/install_ubuntu)

# Docs
- [Workspace Setup Source](https://github.com/athackst/vscode_ros2_workspace)
- [Config Explanations](docs/config-explanations/README.md)
- [Docker Commands](docs/docker-commands/README.md)

# Launch Commands

NB! On Windows, all commands below should be run in a WSL terminal.

## ROS2

A ros2 container for general testing

```bash
docker-compose run ros2
clear
```

When you wish to close the terminal, press `Ctrl + D` or type `exit`.

## ROS2-dev

A ros2 container for package development

Starting the container:
```bash
docker-compose up -d ros2-dev
```
Opening a terminal into the container:
```bash
docker exec -it ros2-dev bash
clear
```
When you wish to close the terminal, press `Ctrl + D` or type `exit`.

## Talker

Publishes messages that the listener should receive if shared memory and network is correctly configured.

```bash
docker-compose up talker
```

## Listener

Listens for messages from the talker. If shared memory and network is correctly configured, it should receive messages from the talker.

```bash
docker-compose up listener
```

## Turtlesim

[Video Tutorial](https://www.youtube.com/watch?v=PlS6YCu5CT4)

Start docker containers

```bash
docker-compose up turtlesim
```

Open a different terminal, enter the turtlesim container and run the control package. The turtle in the other window should move when you press the arrow keys. Make sure to have the terminal with the controls focused when using your arrow keys, not the turtle window.

```bash
docker-compose run turtlesim-control
```

Open another terminal to see your command messages

```bash
docker-compose run ros2 ros2 topic list
docker-compose run ros2 ros2 topic echo turtle1/cmd_vel
```

## Gazebo

```bash
docker-compose up gazebo
```

All files related to the gazebo simulation are located in the `.gazebo` folder which is generated the first time the gazebo container is started.

# Tutorials

- [ROS2 Launch Files](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)
- [ROS2 Docker Image](https://hub.docker.com/_/ros/)
- [Setting up a robot simulation (Gazebo)](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html)
- [Gazebo](https://gazebosim.org/docs)

# ROS Docker Images

- [Official Images](https://hub.docker.com/_/ros/)
- [Meta Images](https://hub.docker.com/r/osrf/ros)
