#! /bin/bash


# Setup the ROS package list to source.list
sudo sh -c 'echo \"deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main\" > /etc/apt/sources.list.d/ros-latest.list'

# Check source.list is added or not
if [! -e /etc/apt/source.list.d/ros-latest.list]; then
	echo "[ERRPR] Unable to setup source.list. Exiting..."
	exit 0
fi

echo "[INFO] Source.list setup already"

