#! /bin/bash

ros_distro=noetic
user_name=${whoami}

echo "[INFO] ROS Noetic Installation Starting..."
echo "[INFO] Checking the Ubuntu Version..."

version='lsb_release -sc'
echo "[INFO] The Ubuntu version is [Ubuntu $version]."

# If the version is not Focal, then quit the process
case $version in 
	"focal" )
	;;
	*)
	echo "[ERROR] The ROS Noetic is primarily targeted at Ubuntu Focal (20.04)."
	exit 0
esac

# Setup the ROS package list to source.list
echo "[INFO] Setup the sources.list..."
sudo sh -c 'echo \"deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main\" > /etc/apt/sources.list.d/ros-latest.list'

# Check source.list is added or not
if [ ! -e /etc/apt/source.list.d/ros-latest.list]; then
	echo "[ERROR] Unable to setup source.list. Exiting..."	
	exit 0
fi
echo "[INFO] Source list setup already."

# Install commandLine URL
echo "[INFO] Checking the curl installation for adding the key..."
sudo apt install curl 

echo "[INFO] Curl configuration done. Adding the keys..."
ret=$(curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -)

# Checking the return values
case $ret in
	"OK" )
    ;;
	*)
	echo "[ERROR] Unable to add the ROS keys."
	exit 0
esac
echo "[INFO] Keys Adding Done."

# Update Ubuntu package index
sudo apt update -y

# Starting to install the ROS Noetic and ask for the install type
echo "[INFO] Installing ROS Noetic. Choosing the install type..."
echo "[1. Desktop-Full Install] | [2. Desktop Install] | [3. ROS-BASE (BARE-BONES)] "

read -p "[Entering] Installation Type (Default to be 1): " ans

case "$ans" in 
	1)
	pkg_type="desktop-full"
	;;
	2)
	pkg_type="desktop"
	;;
	3)
	pkg_type="ros-base"
	;;
	*)
	pkg_type="desktop-full"
	;;
esac
echo "[INFO] Starting Install ROS Noetic $ans ..."

# Starting Installation.
sudo apt-get install -y ros-${ros_distro}-${pkg_type}
echo "[INFO] The ROS Noetic installation Done. Setting the ROS Environment and adding to the .bashrc file..."

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Checking
ros_version='rosversion -d'
echo "[INFO] Installation Finish. The current ROS version is $ros_version. Using [roscore] command to check."