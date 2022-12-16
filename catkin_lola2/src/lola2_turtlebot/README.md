# Adaptations for Turtlebot2 platform

In this section we will cover how to run LOLA scripts in a Turtlebot2 commercial platform

## LOLA Workspace Creation
First of all as we are using the LOLA packages, we must do the basic LOLA installation 
required to work with them. 

It can be done by executing the [full installation](/scripts/full_lola_installation) previously seen:

```bash
sudo bash full_lola_instalation.sh 
```

## Turtlebot Workspace Creation

To start using our ROS packages in your Turtlebot2 you first will need to create different workspaces.
We will create a workspace to initialize the turtlebot, another one to install all Lola packages, and two more of them to use the Astra Orbbec Camera.

For that purpose execute the following [script](/scripts/create_turtlebot_workspace.sh): 

```bash
sudo bash create_turtlebot_workspace.sh
``` 

The mentioned script has the minimal packages need to run the LOLA ones and initialize Turtlebot2.
If its wanted to simulate the Turtlebot, install the following package in the workspace.

```bash
cd ~/turtlebot_ws/src
git clone https://github.com/turtlebot/turtlebot_simulator
cd ..
rm -r build devel
catkin_make
``` 

## Usage for joystick

To run turtlebot with joystick and check what the move discrete package is doing in its inside, follow the next steps:

1. Open a terminal and run the following commands to connect your PC with the Turtlebot2 platform:
```bash
cd ~/lola2/catkin_lola2
. devel/setup.bash
roslaunch turtlebot_bringup minimal.launch
``` 
Turtlebot should make a connection sound after this step.

2. Open a terminal and run the following commands to run the lola packages:
```bash
cd ~/lola2/catkin_lola2
. devel/setup.bash
roslaunch lola2_discrete discrete_move_joy.launch
``` 
You should be able to control Turtlebot now.

## Initialize Orbbec Astra Camera

When Turtlebot packages are installed there are also the Orbbec Astra camera ones. 

To initialize the Orbecc Astra and make it start sending images, run the following command in a new terminal:

```bash
cd ~/lola2/catkin_lola2
. devel/setup.bash
roslaunch astra_camera astra.launch
``` 

Now astra_camera is sending images in a bgr8 format through the "/camera/color/image_raw" topic.