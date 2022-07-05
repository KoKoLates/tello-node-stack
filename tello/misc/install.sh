#! /bin/bash

# TODO list
# 1. version checking 
# 2. libraries installation
# 3. tool installation
# 4. package clone compile ?

echo "Install necessary Python libraries ..."
pip install numpy
pip install av
pip install opencv-python 
pip install opencv-contrib-python 
pip install tellopy
pip install smach

# import sys, select, termios, tty for keyboard twist 