# MOTKIN Gazebo motkin_gz_ros2_control

This plug is a fork of [gz_ros2_control](https://github.com/ros-controls/gz_ros2_control).
It implements the specificities of motkin that are accessible through ros2_control.
It tries to provides a system interface similar to ros2_control_motkin_hardware_interface.

# Installation

## From source
```
colcon build --packages-select motkin_gz_ros2_control
```

## Matrix of compatibility.
The package follows the compatibility matrix specified [here](https://gazebosim.org/docs/latest/ros_installation/)

Then the target is to  try to maintain this package for:
- Jazzy (LTS) - GZ Harmonic
