# LOLA2-Platform setup

## Description
This project is built with ROS and Python. This release provides a whole ROS nodes structure that makes a our LOLA robotic platform able to navigate autonomously using RVIZ.

## Demo
https://user-images.githubusercontent.com/38068010/123829900-1e374500-d903-11eb-9b3e-be9fd4f85a9e.mp4

## Installation
To use this software you must have ROS installed in the platform. We have been using ROS Melodic because our platform runs Ubuntu 18.04. In case that you are runinng Ubuntu 16.04 you have to install ROS Kinetic. 

### ROS Melodic Installation

Setup the sources.list

```shell
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu 
(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

Setup the keys

```shell
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
```

We recommend to install the full version with all the libraries (including RVIZ).
```shell
sudo apt install ros-melodic-desktop-full
```

Environment setup

```shell
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
Dependencies for building packages

```shell
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt install python-rosdep
sudo rosdep init
rosdep update
```

It is also necessary to install libraries that may not be installed with the standard ROS installation:

```shell
sudo apt-get install ros-melodic-spatio-temporal-voxel-layer
sudo apt-get install ros-melodic-navigation
```

### Arduino installation

LOLA platform includes an Arudio to communicate with the motors and encoders. In this repository we include, in the Arduino folder, all the libraries we have developed for this communication. To install our software in your Arduino board you just have to follow the following steps:

1. Download the Arduino IDE, https://www.arduino.cc/en/software


2. Download the mcp2515 library needed to make the software work, https://github.com/autowp/arduino-mcp2515/archive/master.zip


3. Install the library as it is represented in this images

<img width="747" alt="ZIP" src="https://user-images.githubusercontent.com/38068010/134490433-106c346d-ca00-43f9-a3f2-22073fc0d058.png">  <img width="740" alt="InstallZip" src="https://user-images.githubusercontent.com/38068010/134490475-ef554474-924d-4ee4-bfab-92aad78d6578.png">


4. Open, inside the Arduino folder from this repository, the SRE.ino file.

<img width="744" alt="Abrir" src="https://user-images.githubusercontent.com/38068010/134490698-7ade6a8f-3395-4307-bef7-0bbe613ac26f.png">
<img width="742" alt="Abierto" src="https://user-images.githubusercontent.com/38068010/134490704-c8a7e275-f6c0-473b-966d-f3de047e62b1.png">

5. Compile the project and verify that everything is working well.

6. Connect the Arduino board to the computer and open a serial monitor. If you get some data back from the Arduino board that means that the installation has been done successfully.

<img width="776" alt="SerialMonitor" src="https://user-images.githubusercontent.com/38068010/134491145-ba249f16-a126-4aab-831f-e0145cd3385e.png">

7. If it is not working, it might be because you have to select the "Arduino Mega" in the board options and the baudios option has to be at 115200.

### Catkin Workspace Creation

It is necessary to create a catkin workspace to put the project and compile it:

```shell
mkdir -p ~/lola_navigation_ws/src
cd ~/lola_navigation_ws
catkin_make
source devel/setup.bash
```

To make sure that the workspace is correctly overlaid by the configuration script, make sure that the ROS PACKAGE PATH environment variable includes the directory in which it is located, for this we will use the following command:

```shell
echo $ROS_PACKAGE_PATH
```
This command should return something like:

```shell
/home/yourusername/catkin_ws/src:/opt/ros/melodic/share
```

We need to clone a repository to make the rplidar work. This one has to be done in the src folder inside the catkin workspace:

```shell
cd ~/lola_navigation_ws/src
git clone https://github.com/Slamtec/rplidar_ros.git
```
Add Turtlebot packages:

```shell
curl -sLf https://raw.githubusercontent.com/gaunthan/Turtlebot2-On-Melodic/master/install_basic.sh | bash
```

Add logitech f710 joystick package:

```shell
git clone https://github.com/gramuah/logitech_f710_ros.git
```

Add lola2 package:

```shell
cd ~/
git clone git@github.com:gramuah/lola2.git
cp ~/lola2/catkin_lola2/src/lola2_global ~/lola_navigation_ws/src/
```

Finally, go back to the workspace folder and compile the environment:

```shell
cd ~/lola_navigation_ws
catkin_make
```

## Usage for RVIZ

To use this software you must follow the next steps:

Open a terminal

```shell
cd ~/lola_navigation_ws 
. devel/setup.bash
rosrun lola2_global hwinterface_script_lola2
```

Open another terminal

```shell
cd ~/lola_navigation_ws
. devel/setup.bash
roslaunch lola2_global RVIZ_launch.launch
```

We have to start runing the hwinterface_script_lola2.py first cause if not it will be waiting for the RVIZ_launch.launch to finish. After we launch the hwinterface we launch the RVIZ_launch.launch that will start all the nodes structure that we need to start the navigation.

After this we will see in the screen that RVIZ opens.

<img width="1096" alt="Screenshot 2021-06-30 at 10 45 57" src="https://user-images.githubusercontent.com/38068010/123930814-5d5fa780-d990-11eb-8532-db4bb158d046.png">

The platform will be displayed in a predetermined point in the map, to change that we just have to click in the "2D Pose Estimate" option and click in the map where the platform is located.

https://user-images.githubusercontent.com/38068010/123932843-212d4680-d992-11eb-94fb-80aa0db5e3a2.mp4

When we have the platform in where in the real world is we just have to indicate the point where we want the platform to go. To do that we just have to select the "2D Nav Goal" option and click in the map where you want the platform to be. While you click in the map to select the point you have to drag the cursor indicating the orientation that you want the platform to have when it reaches the point.

https://user-images.githubusercontent.com/38068010/123990523-1ee6de80-d9ca-11eb-8975-39ca9b45680e.mp4

## Usage for teleoperation

To use this package you must follow the next steps:

1. Open a terminal and execute the following commands:

```shell
cd ~/lola_navigation_ws
catkin_make
. devel/setup.bash
rosrun lola2_global hwinterface_script_lola2
```

2. Open another terminal and execute these commands:

```shell
cd ~/lola_navigation_ws
catkin_make
. devel/setup.bash
roslaunch lola2_global lola2_teleop.launch
```

3. Open the last terminal and execute the commands:

```shell
cd ~/lola_navigation_ws
catkin_make
. devel/setup.bash
roslaunch lola2_global lola2_keyboard.launch
```

## Usage for joystick

To use this package you must follow the next steps:

1. Open a terminal and execute the following commands:

```shell
cd ~/lola_navigation_ws
. devel/setup.bash
roslaunch lola2_global basic_lola.launch
```

2. Open another terminal and execute these commands:

```shell
cd ~/lola_navigation_ws
. devel/setup.bash
roslaunch logitech_f710_joy_ros joy_teleop.launch

## Configure teleoperation as an startup service

**Before being able to do this, we need to finish this tasks**

- [ ] TODO: include ``basic_lola.launch`` in ``lola2_global`` package.
- [ ] TODO: update ``lola2_package`` in order to include the serial ports selection for lidar and arduino.
- [ ] TODO: update [README.md](README.md) in order to document the new features and installation process.

1. Install robot_upstart package:

```shell
sudo apt-get install ros-melodic-robot-upstart
```

2. Create the system services necessary to run ros nodes on startup:

```shell
rosrun robot_upstart install lola2_global/launch/basic_lola.launch --job basic_lola --symlink --logdir tempa
sudo systemctl daemon-reload && sudo systemctl start basic_lola

rosrun robot_upstart install logitech_f710_joy_ros/launch/joy_teleop.launch --job logitech_joy --symlink --logdir tempu
sudo systemctl daemon-reload && sudo systemctl start logitech_joy
```

3. Modify the ``logitech_joy.service`` file to include a delay such that it does not interfere with the basic lola package during startup:

```shell
sudo vim /etc/systemd/system/multi-user.target.wants/logitech_joy.service
```

Then add ``ExecStartPre=/bin/sleep 10`` before ``ExecStart`` line.

## Support
In the project there is a folder called "config" where the parameter files reside. Therer a a lot of parameters and all of them are optimized for our platfor, you are free to modify any of them. The most important ones are:

- 'max_velocity' and 'min_velocity'
- 'yaw_goal_tolerance' and 'xy_goal_tolerance'
- 'occdist_scale'
- 'sim_granularity'
- 'inflation_radius'
- 'cost_scaling_factor'

The best support that you can get is the wiki of ROS http://wiki.ros.org/Documentation
Anyway here it is my personal email to contact me in case that you need some help sanmasja@gmail.com

## Roadmap
Right now we have two robotic platforms working perfectly using this software. In the future the idea is to improve the movement and times of the robot, cause sometimes it can take a while to calculate where it is exactly in the map. 

Another interesting thing to maybe implement in the future could be moving all this nodes structure to ROS 2 that makes everthing much easier and it could probably improve the efficiency of the platform while navigating.

## Authors and acknowledgment
Jaime Mas Santillán, Francisco Javier Acevedo Rodríguez, Iván Fernández Munilla y Roberto Javier López Sastre.

## License
[MIT](https://choosealicense.com/licenses/mit/)

## Project status
Right now we continue working in the project to improve it.
