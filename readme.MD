# Repository for Remote Arena's hardware

**Description:**  This repo contains several mainly modules to control the remote arena.
Install it into the server dedicated to comunicate with hardware to control.

## Prerrequisites

1. ROS melodic morenia or later
2. Dependencies

## Build

1. Clone this repo in your preferred directory
2. Go to `catkin_ws` directory
3. Run the following commands
```
source /opt/ros/_ROS_DISTRO_/setup.bash
catkin_make
source deve/setup.bash

## Run with
roslaunch arena_mapping startup.launch
```
