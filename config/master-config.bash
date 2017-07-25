#!/bin/bash
 
export PROJ_BASE=$HOME/projects/RobotSim
export M5_PATH=$PROJ_BASE/gem5
source $PROJ_BASE/config/systemc-config.bash
source $PROJ_BASE/config/rosindigo-setup.bash

# ROS variables
# ROS_DISTRO defined in ros indigo config file
#export ROS_DISTRO=indigo
#export ROS_HOME=$PROJ_BASE/ros/$ROS_DISTRO

export LD_LIBRARY_PATH=$M5_PATH/build/ARM:$LD_LIBRARY_PATH
