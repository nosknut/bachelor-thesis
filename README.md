# Bachelor Thesis

## Setup

### Windows (WSL)

- [WSL Setup](docs/setup/wsl/wsl/README.md)
- [Docker Setup](docs/setup/wsl/docker/README.md)

### Ubuntu

- [ROS2 Setup](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

### Common

After cloning the repository, run the following command to ignore changes to the workspace configs:

```bash
bash scripts/git/ignore_local_config_file_changes.bash
```

## Docs

- [Config Explanations](docs/config-explanations/README.md)
- [Docker Commands](docs/docker-commands/README.md)

## Project overview

The project contains 6 ROS2 packages:

- twig
  - The main entry point of the robot. The other packages in the project should not be used directly, but rather be included in launchfiles from this package.
- twig_description
  - Contains the URDF for the robot
- twig_servo
  - Contains the moveit_servo note that allows jogging individual joints through moveit's planning interface.
- twig_teleop
  - Contains two separate nodes that turn joy messages into useful commands for other nodes.
- twig_moveit_config
  - A moveit config package generated by the [Moveit Setup Assistant](https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html). It handles kinematics, path planning, collision avoidance and has an action server for requesting target positions.
- twig_hardware
  - Contains the ros2_control hardware interfaces for the robot, as well as a diagnostics node for debugging that does not use ros2_control.

## Cleaning the workspace

```bash
bash scripts/ros2/purge.bash
```

## Installing dependencies

```bash
bash scripts/ros2/install-dependencies.bash
```

## Building

```bash
bash scripts/ros2/build-packages.bash
```

## Launching

The project contains 3 different launch configurations.
- Normal Operation
  - Requires hardware
  - Based on ros2_control and MoveIt
  - Contains
    - collision, endpoint and singularity avoidance
    - Transform publishers
    - PID controllers
- Diagnostics
  - Requires hardware
  - Contains
    - A single simple hardware node that publishes all sensor data and forwards all commands directly to servos.
    - A teleop node to transform joy messages into servo speed and relay state commands
- ROS2 Development
  - Does not require hardware
  - Identical to Normal Operation
  - Replaces the hardware interfaces with Mocked Components from ros2_control

### Normal Operation

On the robot

```bash
ros2 launch twig robot_moveit.launch.py
ros2 launch twig servo_teleop.launch.py
```

On the GUI pc

```bash
ros2 launch twig joy.launch.py
ros2 launch twig foxglove.launch.py
```

and optionally on the GUI pc

```bash
ros2 launch twig rviz.launch.py
```

### Diagnostics

On the robot

```bash
ros2 launch twig diagnostics.launch.py
```

On the GUI pc

```bash
ros2 launch twig joy.launch.py
ros2 launch twig foxglove.launch.py
```

## ROS2 Development

```bash
ros2 launch twig moveit_demo.launch.py
```

Publish to /twig_joy using the foxglove Joystick node.

## Development

For development, [open the repository in the ROS2 devcontainer](https://code.visualstudio.com/docs/devcontainers/containers), and use the ROS2 task buttons in the status bar to install dependencies and build the project. While in the devcontainer, use the ROS2 Development configuration to run the project and visualize the robots physical state in RVIz. For more detailed information surrounding the development environment, refer to the [original template](https://github.com/athackst/vscode_ros2_workspace) by athackst.

In [Command Palette](https://code.visualstudio.com/docs/getstarted/userinterface#_command-palette):
```
Dev Containers: Reopen in Container
Tasks: Run Task
  ROS2: Install Dependencies
  ROS2: Build Packages
  ROS2: Run Package
```
