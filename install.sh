#!bin/sh

RED='\033[1;31m'
GREEN='\033[1;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

CURR=$PWD

# Update
sudo apt-get update
sudo apt-get upgrade -y

# ROS Melodic
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt install ros-melodic-desktop-full
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt install python-rosdep
sudo rosdep init
rosdep update

# Dependencies
sudo apt-get install python3.8-dev python3-pip -y
python3.8 -m pip install pip setuptools wheel luma.oled adafruit-circuitpython-neopixel-spi python-can

# Update Device Tree Overlay
sudo rm tegra.dtbo tegra.dtbo
sudo cp ~/claw_ws/src/boot/mcp2515.dtbo /boot/

# Install CLAW Workspace
cd ../..
echo "Running Build Job with -O3 Optimizations"
catkin_make -DCMAKE_BUILD_TYPE=Release
echo "source /home/claw/claw_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
