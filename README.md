# ODRI Gazebo odri_gz_ros2_control

This plug is a fork of [gz_ros2_control](https://github.com/ros-controls/gz_ros2_control).
It implements the specificities of ODRI that are accessible through ros2_control.
It tries to provides a system interface similar to ros2_control_odri_hardware_interface.

# Installation

## From source
```
mkdir -p odri_bolt_ws/src
cd odri_bolt_ws/src
git clone https://github.com/stack-of-stacks/odri_gz_ros2_control
git checkout -b your_ros_release
cd ..
colcon build --packages-select odri_gz_ros2_control
```

## Matrix of compatibility.
The package follows the compatibility matrix specified [here](https://gazebosim.org/docs/latest/ros_installation/)

Then the target is to  try to maintain this package for:
- Humble (LTS) - GZ Fortress
- Jazzy (LTS) - GZ Harmonic
- Rolling on Jammy (22.04) / Noble (24.04)

# Branches

master: Targets rolling releases and for now jammy and noble
jazzy: target
humble:
