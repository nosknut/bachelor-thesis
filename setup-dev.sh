source "/opt/ros/${ROS_DISTRO}/setup.bash"
colcon build
echo source "install/local_setup.bash" >> ~/.bashrc
/bin/sh -c "while sleep 1000; do :; done"