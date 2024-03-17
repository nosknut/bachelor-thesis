# https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

sudo apt update -y
sudo apt upgrade -y

if [[ $(locale charmap) != "UTF-8" ]]; then
        sudo apt update -y && sudo apt install locales -y
        sudo locale-gen en_US en_US.UTF-8
        sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
        export LANG=en_US.UTF-8
fi

sudo apt install software-properties-common -y
sudo add-apt-repository universe -y

sudo apt update -y && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update -y

sudo apt install ros-humble-ros-base -y

sudo apt install ros-dev-tools -y
sudo apt install python3-colcon-common-extensions -y
sudo rosdep init
sudo rosdep update

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source /opt/ros/humble/setup.bash
