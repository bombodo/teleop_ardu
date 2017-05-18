# Teleop_ardu

# Getting Started
Built with ROS-kinetic on Ubuntu Xenial

Using default Python2.7

## Pre-reqs
### Main Machine
```
ros-kinetic-ros-base
ros-kinetic-joy
ros-kinetic-rosserial
ros-kinetic-rosserial-arduino
```
### Arduino
```
ros_lib
```
## Installation
To build the project, navigate to the base directory (teleop_ardu/) and run
```
catkin_make
```
To compile and upload to the Arduino
```
catkin_make ardu_out_firmware_move-upload /dev/ttyXXX
```
# Running
```
roslaunch interpret_joy combine.launch
```
# Troubleshooting
Rosserial install not working && anaconda installed on machine
```
Suggested: comment out the anaconda include path from your ~/.bashrc, relog
```
The Main Machine is unable to access the Arduino serial port
```
Suggested: Modify the Arduino Node in src/interpret_joy/launch/combine.launch to appropriate port
```
Wrong controller being used
```
Suggested: Modify joy node in launch file with the appropriate controller input
```
Undesired controller mapping
```
Suggested: Use jstest-gtk to find proper mappings, modify the launch file for preference
```
