# Logitech F710 ROS joy teleop

Fork from [logitech_f710_ros](https://github.com/husarion/logitech_f710_ros). This package is used to control LOLA platform in teleoperation mode.

# Installation

Clone this repo inside your catkin workspace (usually in our convention, lola_navigation_ws):

```shell
cd ~/lola_navigation_ws/src
git clone git@github.com:gramuah/logitech_f710_ros.git
```

Then go back again to the workspace root and execute a catkin_make:

```shell
cd ..
catkin_make

# Activate the environment
source devel/setup.bash
```

## Setup joy

Connect joy via nano USB receiver and make sure it is in **XInput Mode** (switch in front o the pad with letters **D** and **X**, select **X**).

To test if joy works use `jstest /dev/input/js0`.
If the output is:
        
        jstest: No such file or directory

See `ls /dev/input | grep js` and find your joy number.

## Run the node

To run the node with roslaunch, execute the following command:

```shell
# If environment is not active, run: source ~/lola_navigation_ws/devel/setup.bash
roslaunch logitech_f710_joy_ros joy_teleop.launch
```

## Button mapping

|  Button  |              Function              |
|:--------:|:----------------------------------:|
|   `LB`   |           enable driving           |
|   `RB`   |         slow driving mode          |
|   `RT`   | fast driving mode (**not active**) |

If neither `RB` nor `RT` are pressed robot operates in *normal* driving mode.

To drive robot use sticks.

By default angular `Z` is controlled with left stick left/right direction and linear `X` by up/down direction.

---
# ROS node API

ROS node is translating `/joy` topic to `/cmd_vel` topic.


### Publish

- `/cmd_vel` *(geometry_msgs/Twist)*

### Subscribe

- `/joy` *(sensor_msgs/Joy)*

### Parameters

Following parameters change joystick axes mapped to given robot axes of freedom. For more information about parameter values refer to joy package [wiki page](http://wiki.ros.org/joy#Logitech_Wireless_Gamepad_F710_.28DirectInput_Mode.29).

- `~axis_linear_x` *(int, default: 3)* 
- `~axis_linear_y` *(int, default: 2)*
- `~axis_angular_z` *(int, default: 0)*

Robot can be operated at 3 scales of speed depending on pressed buttons. Values in those parameters are m/s for linear movement and rad/s for angular movement.

- `~slow_linear_x` *(double, default: 0.1)*
- `~slow_linear_y` *(double, default: 0.1)*
- `~slow_angular_z` *(double, default: 0.1)*

- `~normal_linear_x` *(double, default: 0.5)*
- `~normal_linear_y` *(double, default: 0.5)*
- `~normal_angular_z` *(double, default: 0.5)*

- `~fast_linear_x` *(double, default: 2.0)*
- `~fast_linear_y` *(double, default: 2.0)*
- `~fast_angular_z` *(double, default: 2.0)*

