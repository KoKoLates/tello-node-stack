#! /bin/bash

echo "ROS noetic installation..."


# Setup the ROS package list to source.list
sudo sh -c 'echo \"deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main\" > /etc/apt/sources.list.d/ros-latest.list'

# Check source.list is added or not
if [ ! -e /etc/apt/source.list.d/ros-latest.list]; then
	echo "[ERROR] Unable to setup source.list. Exiting..."
	exit 0
fi

echo "[INFO] Source list setup already."
echo "[INFO] Curl installation for key adding."

# Install commandLine URL
sudo apt install curl 

# Update Ubuntu package index
sudo apt update 

