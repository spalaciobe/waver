#!/bin/bash

source /opt/ros/$ROS_DISTRO/setup.bash

cd waver_ws && catkin_make

source devel/setup.bash

if [ $1 == "rviz" ]; then
    roslaunch waver_viz rviz.launch
elif [ $1 == "gazebo" ]; then
    roslaunch waver_gazebo gazebo.launch
elif [ $1 == "controller" ]; then
    roslaunch waver_description controller.launch
elif [ $1 == "teleop" ]; then
    rosrun teleop_twist_keyboard teleop_twist_keyboard.py
fi

exec "$@"
