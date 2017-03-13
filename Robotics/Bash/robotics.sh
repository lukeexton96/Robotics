#!/bin/bash

# Start ROSCORE
echo Starting ROSCORE...
roscore &

# open new terminal
# xterm -e command

# update the packages installed on your computer, to get the "training arena"

sudo apt-get update
computing

sudo apt-get dist-upgrade
computing
y

# launch the new training arena (simulation) 
roslaunch uol_turtlebot_simulator object-search-training.launch