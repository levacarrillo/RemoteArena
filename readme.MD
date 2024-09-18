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

sudo apt-get install ros-melodic-web-video-server -y
sudo apt-get install ros-kinetic-usb-cam
sudo apt-get install ros-noetic-video-stream-opencv

sudo apt-get install ros-noetic-roslint -y
sudo apt-get install ros-noetic-rviz-visual-tools -y

sudo apt-get install python3-pip -y
sudo pip install Flask -y
sudo apt-get install python3-venv -y °

sudo pip3 install rospkg

sudo apt-get install ros-noetic-rosserial-python
sudo apt-get install ros-noetic-rosserial-arduino
sudo apt-get install postgresql

sudo pip3 install psycopg2-binary Flask-SQLAlchemy
sudo pip3 install flask-socketio
sudo pip3 install python-dotenv
