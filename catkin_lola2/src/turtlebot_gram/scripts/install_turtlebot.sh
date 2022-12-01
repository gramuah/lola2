
sudo apt install liborocos-kdl-dev -y

rosdep update
rosdep install --from-paths . --ignore-src -r -y

cd ~/turtlebot_ws
catkin_make


##################################
### Orbecc Astra Camera, comment if you dont have one
##################################
cd

# Install dependencies
sudo apt install libgflags-dev  ros-$ROS_DISTRO-image-geometry ros-$ROS_DISTRO-camera-info-manager ros-$ROS_DISTRO-image-transport ros-$ROS_DISTRO-image-publisher libgoogle-glog-dev libusb-1.0-0-dev libeigen3-dev

# Install libuvc
git clone https://github.com/libuvc/libuvc.git
cd libuvc
mkdir build && cd build
cmake .. && make -j4
sudo make install
sudo ldconfig

# Clone code
cd ~/turtlebot_ws/src

git clone https://github.com/orbbec/ros_astra_camera.git

cd ~/turtlebot_ws
catkin_make

# Install udev rules
source ./devel/setup.bash
roscd astra_camera
./scripts/create_udev_rules
sudo udevadm control --reload && sudo  udevadm trigger
~

###########################IMPORTANT###########################
##You should have done a git clone of the lola branch at ~/ before running this part of the script

##In order to make compatible the lola packages with turtlebot youll need to write the following line at the start of the minimal.launch file##
##<remap from="mobile_base/commands/velocity" to="cmd_vel"/>##

##We need the source to be pointing to the turtlebot_ws in order to install turtlebot packages and orbecc astra packages at lola
################################################################
cp ~/lola2/catkin_lola2/src/turtlebot_gram/minimal.launch ~/turtlebot_ws/src/turtlebot/turtlebot_bringup/launch/
cd ~/turtlebot_ws
. devel/setup.bash
cd ~/lola2/catkin_lola2/
rm -r devel build
catkin_make
