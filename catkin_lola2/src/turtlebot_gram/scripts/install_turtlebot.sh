#!/bin/bash


#Installing python-is-python3 to avoid incompatibilities in the python version
#Instalamos python-is-python3 para evitar incompatibilidades entre ficheros
sudo apt install python-is-python3

#Creating the directories where we will generate the workspace for the turtlebot packages
#Creamos los directorios donde generamos el espacio de trabajo para la turtlebot
mkdir -p ~/turtlebot_ws/src
cd ~/turtlebot_ws
catkin_make
cd ~/turtlebot_ws/src



#Installing turtlebot packages and pasting in the src directory
# Descargamos desde github los paquetes para la turtlebot" de noetic
git clone https://github.com/turtlebot/turtlebot.git
git clone https://github.com/turtlebot/turtlebot_msgs.git
git clone https://github.com/turtlebot/turtlebot_apps.git

#This clone is to avoid issues
git clone https://github.com/yujinrobot/yujin_ocs.git

#Exctract the directories that are interesting for us and we remove the leftovers
mv yujin_ocs/yocs_cmd_vel_mux yujin_ocs/yocs_controllers yujin_ocs/yocs_velocity_smoother ./
rm -rf yujin_ocs

#Adding the monitory package
git clone https://github.com/ros-drivers/linux_peripheral_interfaces.git
mv linux_peripheral_interfaces/laptop_battery_monitor ./
rm -rf linux_peripheral_interfaces

#Adding the melodic branch of the kobuki git
git clone https://github.com/yujinrobot/kobuki.git


sudo apt install liborocos-kdl-dev -y
sudo apt install ros-noetic-joy
rosdep update
rosdep install --from-paths . --ignore-src -r -y


#We will need to remap the turtlebot topic in order to use the lola packages in turtlebot
#We will do a remap in order to change the name of the /cmd_vel_mux/input/teleop and use the topic that we need



#Create packages
#Creamos los paquetes
cd ~/turtlebot_ws
catkin_make

###########################IMPORTANT###########################
##In order to make compatible the lola packages with turtlebot youll need to write the following line at the start of the minimal.launch file##
##<remap from="mobile_base/commands/velocity" to="cmd_vel"/>##
################################################################
cp ~/lola2/catkin_lola2/src/turtlebot_gram/launch/minimal.launch ~/turtlebot_ws/src/turtlebot/turtlebot_bringup/launch/



#We need the source to be pointing to the turtlebot_ws in order to have all the packages from the turtlebot a
#and from opt/ros in lola
. devel/setup.bash
###################################
## Catkin Workspace Creation ###
###################################

cd ~/lola2/catkin_lola2/
rm -r devel build
catkin_make

###This part is for the Orbecc Astra camera, comment if you dont have one


sudo apt install libgflags-dev  ros-$ROS_DISTRO-image-geometry ros-$ROS_DISTRO-camera-info-manager\
ros-$ROS_DISTRO-image-transport ros-$ROS_DISTRO-image-publisher libgoogle-glog-dev libusb-1.0-0-dev libeigen3-dev

git clone https://github.com/libuvc/libuvc.git
cd libuvc
mkdir build && cd build
cmake .. && make -j4
sudo make install
sudo ldconfig

mkdir -p ~/orbbec_astra_ws/src
cd ~/orbbec_astra_ws/src
git clone https://github.com/orbbec/ros_astra_camera.git

cd ~/orbbec_astra_ws
catkin_make

cd ~/ros_ws
source ./devel/setup.bash
roscd astra_camera
./scripts/create_udev_rules
sudo udevadm control --reload && sudo  udevadm trigger




cd ~/lola_navigation/
sudo chmod 777 /src/lola2_discrete/script/discrete_move_joy.py
catkin_make
