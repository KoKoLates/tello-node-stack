# Tello Stack
The stack package consists of different nodes for DJI Tello drone. The mainly purpose is to `control` and `navigation` the drone based on the `orb slam` algorithm in the indoor environments. The system is construct in the ROS noetic, and could be visualizede in rviz.


## Dependencies
__1.Tello stack__
```
https://github.com/KoKoLates/Tello-Stack-ROS
```

__2.ORB SLAM2__
```
https://github.com/KoKoLates/ORB-SLAM2-ROS
```

__3.Flowers tracker__ <br>
The flower tracker is used to tracking and counting the flower in the greenhouse. The computation is `offline` in the backend.
```
https://github.com/KoKoLates/Flowers-Tracker
```

## Instruction
__1.Map constrcution__ <br>
Using the tello driver node, orb slam2 and keyboard teleop to construct the point cloud map.
```bash
# laucn the driver and orb slam2 node
roslaunch tello_node tello_orb.launch

# launch the keyboard node
roslaunch keyboard_teleop keyboard_teleop.launch
```
Then ones could using the keyboard to control the drone and flying around the space to construct the map.

__2.Waypoints tracking__ <br>
Tracking the waypoint which is setup already in the `navigation.py` file.
```bash
# laucn the driver and orb slam2 node.
# note that load in the pre-built map and turn into localization mode
roslaunch tello_node tello_orb.launch

# launch the keyboard node
# take off to initialize and localize the position
roslaunch keyboard_teleop keyboard_teleop.launch

# controller interface
# for pid controller and waypoints manage
roslaunch controller_interface controller.launch

# the mission waypoints
rosrun controller_interface navigation.py
```