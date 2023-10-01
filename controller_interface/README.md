# Controller Interface

## 1. Overview
A High level waypoint tracking controller implemented based on ROS noetic. The mission waypoint is defined in `navigation` node or setup in other interactive waypoint management plugin. The `control_interface` will manage the tracking process with PID packages derivatived from ROS. Also, the tracking trajectory could be visualize in ROS rviz.

### Requirements
* [`PID controller node`](https://wiki.ros.org/pid)
* [`Robot localization node`](https://wiki.ros.org/robot_localization) -> optional

### Launch
* `$ roslaunch control_interface controller.launch`
* `$ rosrun control_interface navigation.py`

## 2. Node - Control Interface
### Publish topics
* `/error_x` [std_msgs/Float64](https://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float64.html)
* `/error_y` [std_msgs/Float64](https://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float64.html)
* `/error_z` [std_msgs/Float64](https://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float64.html)
* `/error_yaw` [std_msgs/Float64](https://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float64.html) 
* `/zero_setpoint` [std_msgs/Float64](https://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float64.html)


### Subscribe topics
* `/tf` [tf2_msgs/TFMessage](https://docs.ros.org/en/jade/api/tf2_msgs/html/msg/TFMessage.html)
* `/orb_slam/pose` [geometry_msgs/PoseStamped](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html)

### Services
* `/set_reference`
### Parameters
* `~/pid_pitch/..`
* `~/pid_roll/..`
* `~/pid_yaw/..`
* `~/pid_thrust/..`
* `Kp`、`Ki`、 `Kd` 
* `upper_limit` 、 `lower_limit` 、 `windup_limit`
* `max_loop_frequency` 、 `min_loop_frequency`

## 3. Node - Navigation
### Publish topics
* `/tello/take_off` [std_msgs/Empty](https://docs.ros.org/en/api/std_msgs/html/msg/Empty.html)
* `/tello/land` [std_msgs/Empty](https://docs.ros.org/en/api/std_msgs/html/msg/Empty.html)
* `/tello/cmd_vel` [geometry_msgs/Twist](https://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html)
* `/way_path` [nav_msgs/Path](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html)

### Subscribe topics
* `/orb_slam2_mono/pose` [geometry_msgs/PoseStamped](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html)
* `/pid_pitch/control_effort` [std_msgs/Float64](https://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float64.html)
* `/pid_roll/control_effort` [std_msgs/Float64](https://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float64.html)
* `/pid_yaw/control_effort` [std_msgs/Float64](https://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float64.html)
* `/pid_thrust/control_effort` [std_msgs/Float64](https://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float64.html)

### Service proxy
* `/set_reference`

## 4. Node - Path Visualizer
### Publish topics
* `/path_visualizer/ekf_path` [nav_msgs/Path](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html)
* `/path_visualizer/orb_path` [nav_msgs/Path](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html)
### Subscribe topics
* `/tf` [tf2_msgs/TFMessage](https://docs.ros.org/en/jade/api/tf2_msgs/html/msg/TFMessage.html)
* `/orb_slam/pose` [geometry_msgs/PoseStamped](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html)

## 5. Features
### Cooperate with ORB SLAM2 node
ORB SLAM2 node could help the drone localize in the pose and position. Through the difference between `current pose` and `target pose`, the controller could obtain the pose error and give the feedback to operating the drone behavior.
* `$ roslaunch orb_slam2_ros tello.launch`