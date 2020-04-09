# Auto-Nav

Auto navigation for turtlebot with ROS navigation stack.

## Dependancy

This package relies on `turtlebot3_bringup`, `turtlebot3_slam` and `turtlebot3_navigation` to work. Make sure they are available in the catkin workspace. The package should be under `catkin_ws`. After copying, invoke `catkin_make` to have everything indexed.

C++ extension in this package is NOT added to CMakeLists.txt, thus cannot be addressed by `catkin_make`. To build, run `make` at the package root directory (where this readme file is in). It requires GNU Make and GCC with C++11 support.

## Usage

After starting `roscore` and bringing up the turtlebot, run
```
roslaunch auto_nav auto_nav.launch
```
to bring up the environment. Do NOT run `turtlebot3_slam` or `turtlebot3_navigation` separately - the launch file in this package will handle them.

If no error is shown, run the script with
```
python2 scripts/auto_nav/auto_nav.py
```
from the package root.