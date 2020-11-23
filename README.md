<h1 align=center> Feature Tracker </h1>

[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

## About
- Kapil Rawal - Robotics Engineer. My area of interests are general/aerial robotics, computer vision, and path planning.
- Email Id: Kapilrawal1995@gmail.com

## Project Overview
The project is a feature tracker implementation using ROS and OpenCV trackers. In the current verison only pyhton script is tested for feature tracking (check output in video folder). The cpp files contains the similar framework but is not tested.

The project currently subscribe to /camera/image/compressed topic and publishes at /output/feature_image/compressed

## License
This project is under the [BSD License](https://https://github.com/krawal19/feature_tracker/blob/main/LICENSE).

## Dependencies
The project requires the following dependenices:
- Gcc 7.5.0
- ROS Melodic
- Python 2
- OpenCV 3.2
- OpenCV_contrib
- CMake
- Ubuntu 18.04

## Program installation
```
mkdir catkin_ws
cd catkin_ws
mkdir src
cd src
git clone https://github.com/krawal19/feature_tracker.git
cd ..
catkin build
source devel/setup.bash
```
## Running code
- Start roscore
- Run the below command
```
rosrun feature_tracker feature_tracker.py
```
- Play bag file for getting input compressed image