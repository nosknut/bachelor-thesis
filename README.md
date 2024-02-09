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

# Development

The following are ROS2 commands that can be run inside the ros2 docker container. Open a ros2 terminal with the following commands:

```bash
docker-compose run --rm ros2
clear
```

## Install Depencencies

```bash
rosdep install --from-paths src --ignore-src -y --rosdistro=$ROS_DISTRO
```

## Build Package

```bash
colcon build
```

## Source the Workspace

```bash
source install/setup.bash
```

## Launch Rviz

```bash
ros2 launch auv_manipulator rviz_launch.xml
```

## Generate URDF from xacro

Visualize the urdf model [here](https://mymodelrobot.appspot.com/)

```bash
xacro manipulator.urdf.xacro > manipulator.urdf
```

## Start Moveit2 container

```bash
docker-compose run --rm moveit2
```

# Quick Commands

## Build Package

```bash
docker-compose run --rm ros2
colcon build
source install/setup.bash
```

## Open Sourced Terminal

```bash
docker-compose run --rm ros2
source install/setup.bash
clear
```

## Launch Rviz

```bash
docker-compose run --rm ros2
source install/setup.bash
clear
ros2 launch auv_manipulator rviz_launch.xml
```

## Launch Moveit2 Tutorial

Command for launching [this](https://moveit.picknik.ai/main/doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial.html) moveit2 tutorial.

```bash
docker-compose run --rm moveit2 ros2 launch moveit2_tutorials demo.launch.py
```

## Launch Moveit2 Setup Assistant

Command for launching [this](https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html) tutorial.

```bash
docker-compose run --rm moveit2 ros2 launch moveit_setup_assistant setup_assistant.launch.py
```


# Launch Commands

NB! On Windows, all commands below should be run in a WSL terminal.

## ROS2

```bash
docker-compose run --rm ros2
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
docker-compose run --rm turtlesim-control
```

Open another terminal to see your command messages

```bash
docker-compose run --rm ros2 ros2 topic list
docker-compose run --rm ros2 ros2 topic echo turtle1/cmd_vel
```

## Gazebo

```bash
docker-compose up gazebo
```

All files related to the gazebo simulation are located in the `.gazebo` folder which is generated the first time the gazebo container is started.

# Tutorials
- [ROS2 Cheat Sheet](https://github.com/ubuntu-robotics/ros2_cheats_sheet/blob/master/cli/cli_cheats_sheet.pdf)
- [ROS2 Launch Files](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)
- [ROS2 Docker Image](https://hub.docker.com/_/ros/)
- [Setting up a robot simulation (Gazebo)](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html)
- [Gazebo](https://gazebosim.org/docs)
- [Movit2](https://moveit.picknik.ai/main/doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial.html)
  - [ros2_control and state publishers](https://moveit.picknik.ai/main/doc/how_to_guides/moveit_launch_files/moveit_launch_files_tutorial.html)
  - [servoing](https://moveit.picknik.ai/humble/doc/examples/realtime_servo/realtime_servo_tutorial.html)
  - [Gamepad](https://moveit.picknik.ai/main/doc/how_to_guides/controller_teleoperation/controller_teleoperation.html)
  - [Simulations](https://moveit.picknik.ai/main/doc/how_to_guides/isaac_panda/isaac_panda_tutorial.html)
  - [pose_tracking_with_lowpass_filter](https://github.com/ros-planning/moveit2_tutorials/blob/main/doc/examples/realtime_servo/launch/pose_tracking_tutorial.launch.py)
  - [preception](https://github.com/ros-planning/moveit2_tutorials/blob/main/doc/examples/perception_pipeline/perception_pipeline_tutorial.rst)
  - [urdf anbd sdrf](https://github.com/ros-planning/moveit2_tutorials/blob/main/doc/examples/urdf_srdf/urdf_srdf_tutorial.rst)
- [ros2_controll_demos](https://github.com/ros-controls/ros2_control_demos)
- [ros2_control](https://control.ros.org/master/index.html)
  - [6dof robot](https://control.ros.org/master/doc/ros2_control_demos/example_7/doc/userdoc.html)

# ROS Docker Images

- [Official Images](https://hub.docker.com/_/ros/)
- [Meta Images](https://hub.docker.com/r/osrf/ros)
