bash scripts/ros2/update-registries.bash
bash scripts/ros2/clone-repos.bash
rosdep install --from-paths src --ignore-src -y --rosdistro=$ROS_DISTRO