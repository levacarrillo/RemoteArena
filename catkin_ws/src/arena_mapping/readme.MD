# ARENA_MAPPING PACKAGE

This package contains the nodes used to create a map of the real arena and locate the robot inside of it, using the OpenCV tools with aruco's codes in specific points.

Inside the directory are set the .launch files using to start several nodes to control and visualize the remote arena.

## Nodes:
1. `aruco_detector.py`
2. `obstacles_detector.py`

## Nodes description
`aruco_detection.py`: This python script is used for detecting all the aruco's codes on the remote arena and the robots. With all the arucos detected, the script computes all the positionsin real time.

`obstacles_detector.py`: This python script is used for detecting the obstacles inside the remote arena used for reactive beheavors. This node uses the OpenCV tools for tracking color and publish lines or positions to be able to print them in the simulator.
