#!/bin/bash

# Open appropriate web pages
if [ -n $BROWSER ]; then
  $BROWSER 'https://github.com/lukeexton96/Robotics'
elif which xdg-open > /dev/null; then
  xdg-open 'https://github.com/LCAS/teaching/wiki/CMP3103M'
elif which gnome-open > /dev/null; then
  gnome-open 'http://wwww.google.com'
else
  echo "Could not detect the web browser to use."
fi


# Start ROSCORE
echo Starting ROSCORE...
roscore &

# Open another console tab
# (COMMANDS HERE)

# Need to open another tab to run these on
# update the packages installed on your computer, to get the "training arena"
sudo apt-get update
computing

sudo apt-get dist-upgrade
computing
y

# launch the new training arena (simulation) 
roslaunch uol_turtlebot_simulator object-search-training.launch &

# open new terminal

# launch spyder
spyder &

# Maybe? Open python script too
