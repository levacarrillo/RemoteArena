#!/bin/bash
# Basic instalation script

sudo apt-get update -y 
sudo apt-get upgrade -y
sudo apt-get install ros-noetic-web-video-server -y
sudo apt-get install ros-noetic-usb-cam -y
sudo apt-get install ros-noetic-video-stream-opencv -y
sudo apt-get install ros-noetic-roslint -y
sudo apt-get install ros-noetic-rviz-visual-tools -y
sudo apt-get install ros-noetic-map-server -y
sudo apt-get install ros-noetic-arudo-ros -y
sudo apt-get install ros-noetic-plotjuggler-ros -y

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
