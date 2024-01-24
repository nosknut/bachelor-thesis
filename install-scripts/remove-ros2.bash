# https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

sudo apt remove ~nros-humble-* -y && sudo apt autoremove -y

sudo rm /etc/apt/sources.list.d/ros2.list
sudo apt update -y
sudo apt autoremove -y
# Consider upgrading for packages previously shadowed.
sudo apt upgrade -y