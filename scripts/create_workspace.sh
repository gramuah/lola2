#!/bin/bash

###################################
#### Catkin Workspace Creation ####
###################################

mkdir -p ~/lola_navigation/src

cd ~/lola_navigation/src

# Rplidar package
git clone https://github.com/Slamtec/rplidar_ros.git

# Lola2 package 
cd ~/
git clone https://github.com/gramuah/lola2.git
cp -r ~/lola2/catkin_lola2/src/lola2_global ~/lola_navigation/src
cp -r ~/lola2/catkin_lola2/src/lola2_discrete ~/lola_navigation/src
cp -r ~/lola2/catkin_lola2/src/logitech_f710_ros ~/lola_navigation/src
sudo rm -rf ~/lola2

sudo apt install ros-noetic-joy -y

sudo apt install python3-pip -y
pip install pyserial

cd ~/lola_navigation/
sudo chmod 777 /src/lola2_discrete/script/discrete_move_joy.py
catkin_make
