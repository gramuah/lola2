# discrete_move package

## Description
This ROS package is designed to control the Lola Robot through parameterized movements, allowing the user
to specify the distance and angle of each movement. The package supports four possible movements:
  * **Forward**
  * **Backward**
  * **Turn left**
  * **Turn right**

The forward and backward movements can be controlled by specifying the distance that the robot needs to travel, 
while the turn left and turn right movements can be parameterized with the exact angle of movement required.

Futhermore, a ROS server has been designed to enable other ROS packages to connect as clients and execute parameterized discrete movements.

In summary, this ROS package provides a powerful tool for controlling the Lola Robot with a high degree of  precision, 
making it an excellent choice for a wide range of applications.

# Dependecies
This package has been developed in Python3 and requires the following dependencies:
* ROS Noetic. 
* Math library

# Installation

To use this package you must follow the next steps:

1. Open a terminal and compile the package.

```shell
cd ~/catkin_ws
catkin_make
```

2. Load the package into the ROS environment.

```shell
. devel/setup.bash
```

# Usage
To use this package, you must launch the robot control node:
```shell
rosrun discrete_move discrete_server.launch
```

# Test
To ensure the proper functionality of this ROS package, two tests have been developed that simulate the robot's movement. 
### Test 1: Make a square.
This test checks if the robot is able to perform a square movement correctly. To run the test, execute the following command:
```shell
rosrun discrete_move test_move_square.py
```
If the robot is able to move in a perfect square, the test will be pass.

### Test 2: Make a rhombus.
This test checks if the robot is able to perform a rhombus movement correctly with parameterized movements. 
To run the test, execute the following command:
```shell
rosrun discrete_move test_move_param.py
```

If the robot is able to move in a perfect rhombus, the test will be pass.

By running these tests, users can verify that the robot is capable of executing the desired movements accurately and precisely. 
If any errors or unexpected behavior are observed, users can use the feedback from the tests to troubleshoot and resolve any issues.

## Authors and acknowledgment
Rafael Flor Rodríguez, Pablo Ríos Navarro y  Francisco Javier Acevedo Rodríguez.

## License
[MIT](https://choosealicense.com/licenses/mit/)

## Project status
Right now we continue working in the project to improve it.
