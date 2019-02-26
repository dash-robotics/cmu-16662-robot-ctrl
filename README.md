# Overview
This package contains control software and utilities for using the robots for the CMU 16-662: Robot Autonomy course.

# Dependencies
This package requires the following software dependencies:
- Ubuntu 16.04 LTS
- ROS Kinetic
- TBD

# Installation
```
mkdir -p ~/ws/robauton_lab1/src
cd ~/ws/robauton_lab1
catkin_make
cd ~/ws/robauton_lab1/src
git clone https://github.com/timeous/cmu-16662-robot-ctrl
cd ~/ws/robauton_lab1
catkin_make
```

# Basic Usage
After successful installation and build:
```
source ~/ws/robauton_lab1/devel/setup.bash
cd ~/ws/robauton_lab1/cmu-16662-robot-ctrl
roslaunch launch/position_control.launch
```

While this is running, in a separate terminal window (**NOTE**: the robot will move after these commands):
```
source ~/ws/robauton_lab1/devel/setup.bash
cd ~/ws/robauton_lab1/cmu-16662-robot-ctrl
python scripts/command_joints.py
```

# Troubleshooting
The argument `device_name` in `position_control.launch` needs to be updated to the actual device being used on your platform, which is the USB device that the U2D2 controller is using. If the default parameter in `position_control.launch` is not valid, the following command will show the device name:
```
dmesg | grep 'attached to tty'
```
The message will appear similar to:
```
[    9.602622] usb 1-3: FTDI USB Serial Device converter now attached to ttyUSB0
```
In this case, `ttyUSB0` is indeed the device name. Change your launch script as appropriate based on the actual device your platform is using.

# Attribution
Thanks to Adithya Murali (@adithyamurali) for creating the skeleton and basic usage scripts.
