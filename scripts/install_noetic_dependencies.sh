#!/bin/bash

## Ubuntu 20.04 install of Ros Noetic ##

# Setup your sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Set up your keys
sudo apt install curl -y
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Installation of Ros Noetic Desktop Full
sudo apt update -y
sudo apt install ros-noetic-desktop-full -y

# Environment setup
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Dependencies for building packages
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y

sudo rm -rf /etc/ros/rosdep/sources.list.d/20-default.list

# Initialize rosdep
sudo rosdep init
rosdep update

## Install dependencies for Lola2 ##
sudo apt-get install ros-noetic-spatio-temporal-voxel-layer ros-noetic-navigation -y

sudo apt install ros-noetic-joy python3-pip python-is-python3 -y
pip install pyserial

#Assign symlinks to serial devices
sudo echo 'ACTION=="add", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0666", GROUP:="dialout", SYMLINK+="arduino"
ACTION=="add", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout", SYMLINK+="lidar"' > ./99-local.rules
sudo mv 99-local.rules /etc/udev/rules.d