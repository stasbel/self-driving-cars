#!/usr/bin/env bash

catkin_make
source devel/setup.bash
roslaunch src/follower/main.launch
