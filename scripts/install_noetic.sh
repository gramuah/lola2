#!/bin/bash

############################################
#### Ubuntu 20.04 install of Ros Noetic ####
############################################

# Setup your sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Set up your keys
sudo apt install curl -y
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Installation
sudo apt update -y
sudo apt install ros-noetic-desktop-full -y

# Environment setup
#source /opt/ros/noetic/setup.bash

# Comment if you have more than one ROS distribution installed
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Dependencies for building packages
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y

# Initialize rosdep
sudo rosdep init
rosdep update

sudo apt-get install ros-noetic-spatio-temporal-voxel-layer ros-noetic-navigation -y