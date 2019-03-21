# Overview
This package contains control software and utilities for using the robots for the CMU 16-662: Robot Autonomy course.

# Dependencies
This package requires the following software dependencies:
- Ubuntu 16.04 LTS
- ROS Kinetic

# Installation
```
mkdir -p ~/ws/robauton_example/src
cd ~/ws/robauton_example
catkin_make
cd ~/ws/robauton_example/src
git clone https://github.com/timeous/cmu-16662-robot-ctrl
cd ~/ws/robauton_example
catkin_make
```

# Basic Control Usage
After successful installation and build:
```
source ~/ws/robauton_example/devel/setup.bash
cd ~/ws/robauton_example/src/cmu-16662-robot-ctrl
roslaunch launch/position_control.launch
```

While this is running, in a separate terminal window (**NOTE**: the robot will move after these commands):
```
source ~/ws/robauton_example/devel/setup.bash
cd ~/ws/robauton_example/src/cmu-16662-robot-ctrl
python scripts/command_joints.py
```

The robot can be safely shut down by pressing Ctrl+c on the terminal that is running `position_control.launch`. The robot will move into a safe joint configuration before removing motor power.

# Safety
1. Never stand or place objects within the working space of the robot.
2. Always assume the robot will behave unexpectedly and antagonistically. Plan accordingly so that your experiments are always conducted safely.
3. If you need to power cycle the motors, read the Motor Lockup section below **BEFORE** power cycling.
4. Before running experiments, know exactly what commands to run to expediently shut down the robot. (Ctrl+c in the terminal running the `position_control.launch` script.)

# Troubleshooting

## Can't Connect to Device
The argument `device_name` in `position_control.launch` needs to be updated to the actual device being used on your platform, which is the USB device that the U2D2 controller is using. If the default parameter in `position_control.launch` is not valid, the following command will show the device name:
```
dmesg | grep 'attached to tty'
```
The message will appear similar to:
```
[    9.602622] usb 1-3: FTDI USB Serial Device converter now attached to ttyUSB0
```
In this case, `ttyUSB0` is indeed the device name. Change your launch script as appropriate based on the actual device your platform is using.

## Can't Open Serial Port
The device (default `ttyUSB0`) needs admin rights to open a serial connection to the U2D2. If the device has insufficient permissions, you will receive a similar error message as below:
```
================================================================================REQUIRED process [position_control-2] has died!
process has died [pid 2775, exit code -11, cmd /home/telee/ws/robauton_example/devel/lib/dynamixel_workbench_controllers/position_control __name:=position_control __log:=/home/telee/.ros/log/6f8e199e-44d9-11e9-af4e-7470fd1b9301/position_control-2.log].
log file: /home/telee/.ros/log/6f8e199e-44d9-11e9-af4e-7470fd1b9301/position_control-2*.log
Initiating shutdown!
================================================================================
```
The following terminal command (that provides admin rights to this device) should fix this issue:
```
sudo chmod a+rw /dev/ttyUSB0
```
Substitute `/dev/ttyUSB0` with the actual device name being used, if different.

## Motor Lockup
If the robot stops following the joint commands, you may need to power cycle the motors. Hold the robot arm **BEFORE YOU DISCONNECT MOTOR POWER** because the arm will collapse otherwise. As you hold the arm, carefully unplug the USB controller and the power adapter. Put the arm back in a safe configuration, and reconnect both the controller and the power adapter.

# Attribution
Thanks to Adithya Murali (@adithyamurali) for creating the skeleton and basic usage scripts.
