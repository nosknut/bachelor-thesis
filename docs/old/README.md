# Setup Windows

## WSL

Running gui applications like rviz and gazebo in docker on windows is not practical due to the lack of support for GPU passthrough. The following instructions are for installing and using windows subsystem for linux (WSL), which allows you to run linux gui applications directly in windows.

### Install WSL

- Follow the instructions in [this](https://youtu.be/AMlaEFaKG88?si=Uk1TH-ulXn2jgdA9) video or [this](https://docs.microsoft.com/en-us/windows/wsl/install-win10) website to install WSL.
- Make sure WSL2 is the default version
```bash
wsl --set-default-version 2
```
- [Install Ubuntu WSL](https://apps.microsoft.com/detail/9PDXGNCFSCZV?hl=en-us&gl=US) from the Microsoft Store.
- Run the setup by launching the Ubuntu app from the start menu
- [Configure WSL GUI Support](https://learn.microsoft.com/en-us/windows/wsl/tutorials/gui-apps)
```bash
wsl --shutdown
```

### Launch wsl terminal from VSCode

![1705884512881](docs/assets/launch-wsl-terminal-from-vscode.png)

## Docker

Docker is a service that allows you to run applications in containers without installing the programs on your actual machine. This allows for a standardized environment for running applications, and makes it easier to share code with others. It also takes a lot of debugging off your hands when working across different computers and team members.

### Install Docker

Follow the instructions on [this](https://docs.docker.com/desktop/install/windows-install/) website to install docker.

### Configure Docker in WSL

Follow the instructions on [this](https://docs.docker.com/desktop/wsl/) website to make docker commands available in the WSL terminal.

# Native ROS2

- [Install ROS2](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

# Native WSL Gazebo

The reccomended version of gazibo for ROS2 Humble is Gazebo Ignition Fortress.

Running Gazebo in docker works and is practical when working on many computers, however due to the lack of GPU passthrough, the performance is not great. The following instructions are for running Gazebo natively in WSL.

- [Install WSL](#install-wsl)
- [Open a WSL terminal](#launch-wsl-terminal-from-vscode)
- [Install conda](https://docs.conda.io/projects/miniconda/en/latest/) using the commands below
```bash
mkdir -p ~/miniconda3
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda3/miniconda.sh
bash ~/miniconda3/miniconda.sh -b -u -p ~/miniconda3
rm -rf ~/miniconda3/miniconda.sh

~/miniconda3/bin/conda init bash
~/miniconda3/bin/conda init zsh
```
- [Install Gazebo](https://gazebosim.org/docs/fortress/install_windows) using the commands below
  - Find the latest version
```bash
conda search libignition-gui* --channel conda-forge
conda search libignition-gazebo* --channel conda-forge
```
  - Install the latest version
```bash
conda install libignition-gui6 --channel conda-forge
conda install libignition-gazebo6 --channel conda-forge
```

### Run gazebo on cpu
- https://github.com/gazebosim/gz-sim/issues/1841
```bash
ign gazebo --render-engine ogre
```
- [Gazebo WSL fix issua](https://github.com/gazebosim/gz-sim/issues/920#issuecomment-1767629170)

## Create symlink to project in WSL home directory

Gecause gazebo will be operating in its own environment, it is practical to create a shortcut the project folder in the WSL home directory. This way, you can easily access the project folder from the WSL terminal.

```bash
ln -s /mnt/c/Users/username/path/to/<project-name> ~/<project-name>
```

# Launch Commands

## Gazebo Native

NB! The following commands must be run from a WSL terminal.

```bash
ign gazebo
```

## Ros

A ros2 container for general testing

```bash
docker-compose run ros
```

## Talker

```bash
docker-compose up talker
```

## Turtlesim

NB! The following commands must be run from a WSL terminal.

[Video Tutorial](https://www.youtube.com/watch?v=PlS6YCu5CT4)

Start docker containers

```bash
docker-compose up turtlesim
```

Open a different terminal, enter the turtlesim container and run the control package. The turtle in the other window should move when you press the arrow keys. Make sure to have the terminal with the controls focused when using your arrow keys, not the turtle window.

```bash
docker exec -it turtlesim bash
ros2 run turtlesim turtle_teleop_key
```

Open another terminal to see your command messages

```bash
docker exec -it turtlesim bash
ros2 topic list
ros2 topic echo turtle1/cmd_vel
```

## Gazebo

NB! The following commands must be run from a WSL terminal.

Starting the container

```bash
docker-compose up gazebo
```

All files related to the gazebo simulation are located in the `.gazebo` folder which is generated the first time gazebo is started.

# Config Details

This section explains the purpose of some important configs in the project.

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

# Useful commands

## Docker

### Show running compose services

```bash
docker-compose ps
```

### Show all docker-compose services

```bash
docker-compose config --services
```

### Show all docker images

```bash
docker images
```

### Start all docker-compose services

```bash
docker-compose up
```

### Stop all docker-compose services

```bash
docker-compose down
```

### Start docker-compose service

```bash
docker-compose up service
```

### Stop docker-compose service

```bash
docker-compose down service
```

### Start docker-compose service in detached mode

Starts a docker-compose service in the background so that the terminal is not blocked.

```bash
docker-compose up -d service1 service2 service3
```

### Open terminal in running docker-compose service

```bash
docker-compose exec service bash
```

or

```bash
docker exec -it service /bin/bash
```

### Run docker-compose service with a command

```bash
docker-compose run service command
```

### Run docker-compose service that does not have a command

For containers that have no startup command, this will open the terminal inside the container.

```bash
docker-compose run service
```

### Delete docker imagges matching pattern

```bash
docker rmi $(docker images -a | grep "pattern" | awk '{print $3}')
```

### List all docker containers including inactive
```bash
docker ps -a
```

### List all docker container ids
```bash
docker ps -aq
```

## Remove all docker containers

```bash
docker rm $(docker ps -aq)
```

### List all docker image ids
```bash
docker ps -aq
```

## Remove all docker images

```bash
docker rmi $(docker images -aq)
```

## Remove everything from docker

```bash
docker system prune
```

# ROS Docker Images
- [Official Images](https://hub.docker.com/_/ros/)
- [Meta Images](https://hub.docker.com/r/osrf/ros)

# ROS Docs
- [extends](https://docs.docker.com/compose/compose-file/05-services/#extends)

# Tutorials

- [ROS2 Launch Files](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)
- [ROS2 Docker Image](https://hub.docker.com/_/ros/)
- [Setting up a robot simulation (Gazebo)](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html)
- [Gazebo](https://gazebosim.org/docs)

# Sources

- https://hub.docker.com/_/ros/

- Longest EOL distro at the time of writing this document (20.01.2024) is ros2 `Humble Hawksbill`
  - https://docs.ros.org/en/rolling/Releases.html
- https://docs.sevenbridges.com/docs/mount-a-usb-drive-in-a-docker-container
- https://wiki.ros.org/joy
- https://www.homeautomationguy.io/blog/docker-tips/accessing-usb-devices-from-docker-containers
- https://moveit.picknik.ai/main/index.html
- https://www.reddit.com/r/ROS/comments/q2bf3i/gazebo_running_through_wsl2_in_windows_11_natively/
- https://youtu.be/f8_nvJzuaSU?si=8P1tadQlEiZQ4ZzD
- WSL2 Docker: https://docs.docker.com/desktop/wsl/
- XServer: https://www.youtube.com/watch?v=DW7l9LHdK5c&list=LL&index=3
- https://gazebosim.org/api/gazebo/4.0/gui_config.html
- During ign harmonic they renamed it back to gz: https://gazebosim.org/docs/harmonic/migration_from_ignition

[<-- Back](/README.md)

# Observations
## Gazebo

- [Gazebo Development Chart](https://gazebosim.org/about)
- [Gazebo compatability chart](https://gazebosim.org/docs/fortress/ros_installation)
- Ignition Fortress works well in WSL through a docker container, but crashes in native WSL with the error below. Ignition Fortress is the reccomended version for ROS2 Humble.
```bash
ign gazebo
QStandardPaths: wrong permissions on runtime directory /run/user/0/, 0755 instead of 0700
libEGL warning: MESA-LOADER: failed to open vgem: /usr/lib/dri/vgem_dri.so: cannot open shared object file: No such file or directory (search paths /usr/lib/x86_64-linux-gnu/dri:\$${ORIGIN}/dri:/usr/lib/dri, suff
ix _dri)

libEGL warning: NEEDS EXTENSION: falling back to kms_swrast
terminate called after throwing an instance of 'Ogre::UnimplementedException'
  what():  OGRE EXCEPTION(9:UnimplementedException):  in GL3PlusTextureGpu::copyTo at /build/ogre-next-UFfg83/ogre-next-2.2.5+dfsg3/RenderSystems/GL3Plus/src/OgreGL3PlusTextureGpu.cpp (line 677)
Stack trace (most recent call last) in thread 38579:
#10   Object "[0xffffffffffffffff]", at 0xffffffffffffffff, in
#9    Object "/lib/x86_64-linux-gnu/libc.so.6", at 0x7fa28034684f, in
#8    Object "/lib/x86_64-linux-gnu/libc.so.6", at 0x7fa2802b4ac2, in
#7    Object "/lib/x86_64-linux-gnu/libQt5Core.so.5", at 0x7fa27ab9999d, in
#6    Object "/lib/x86_64-linux-gnu/libQt5Core.so.5", at 0x7fa27ab97f90, in qTerminate()
#5    Object "/lib/x86_64-linux-gnu/libstdc++.so.6", at 0x7fa27c854276, in std::terminate()
#4    Object "/lib/x86_64-linux-gnu/libstdc++.so.6", at 0x7fa27c85420b, in
#3    Object "/lib/x86_64-linux-gnu/libstdc++.so.6", at 0x7fa27c848b9d, in
#2    Object "/lib/x86_64-linux-gnu/libc.so.6", at 0x7fa2802487f2, in abort
#1    Object "/lib/x86_64-linux-gnu/libc.so.6", at 0x7fa280262475, in raise
#0    Object "/lib/x86_64-linux-gnu/libc.so.6", at 0x7fa2802b69fc, in pthread_kill
Aborted (Signal sent by tkill() 38479 0)
```