#!/bin/bash
# Basic instalation script

sudo apt-get update -y 
sudo apt-get upgrade -y
sudo apt-get install arduino-mk -y
sudo apt-get install ros-noetic-cv-bridge -y
sudo apt-get install ros-noetic-image-transport -y
sudo apt-get install ros-noetic-tf -y
sudo apt-get install ros-noetic-tf2-tools -y
sudo apt-get install ros-noetic-tf2-geometry-msgs -y
sudo apt-get install ros-noetic-rosserial-python -y

sudo apt-get install ros-noetic-web-video-server -y
sudo apt-get install ros-noetic-usb-cam -y
sudo apt-get install ros-noetic-video-stream-opencv -y
sudo apt-get install ros-noetic-roslint -y
sudo apt-get install ros-noetic-rviz-visual-tools -y
sudo apt-get install ros-noetic-map-server -y
sudo apt-get install ros-noetic-aruco-ros -y
sudo apt-get install ros-noetic-plotjuggler-ros -y
sudo apt-get install ros-noetic-robot-state-publisher -y

sudo apt-get install python3-pip -y
sudo apt-get install python3-venv -y
sudo pip3 install flask
sudo pip3 install rospkg
sudo apt-get install ros-noetic-rosserial-python -y
sudo apt-get install ros-noetic-rosserial-arduino -y
sudo apt-get install postgresql -y
sudo pip3 install psycopg2-binary
sudo pip3 install flask-sqlalchemy
sudo pip3 install python-dotenv
sudo pip3 install flask-cors
