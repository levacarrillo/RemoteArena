# ARENA_CAMERA PACKAGE

This package contains mainly the node used for connectig with the arena's camera connected to server. The default camera id is set in 0, use `rosparam set /camera_id _NUMBER_` to change it via command line or changeit into launch file.

**IMPORTANT:** This node publishes images at a frequency of 30 Hz. consider it for futures implementations.
