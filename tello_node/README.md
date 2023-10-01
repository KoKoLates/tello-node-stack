# Tello Node

## 1. Overview
A driver node for communicating with dji tello drone using official [software develop kit](https://dl-cdn.ryzerobotics.com/downloads/Tello/Tello%20SDK%202.0%20User%20Guide.pdf) and other unofficial [package library](https://github.com/hanyazou/TelloPy). The unofficial libraries originated from the reverse-engineering the raw packages broadcasted by the Tello. This package is build on top of the unofficial library and mainly for ROS noetic version.

### Build
* `$ cd catkin_ws/src`
* `& git clone https://github.com/KoKoLates/tello-node-stack.git`
* `$ $ cd .. & catkin_make`
* `$ source devel/setup.bash`

### Launch
* Turn on tello drone.
* Connecting to the Wi-Fi access point (`TELLO_XXXXXX`) of tello drone.
* `$ roslaunch tello_node tello_keyboard.launch`

## 2. Node
### Publish topics
* `/tello/image_raw` [sensor_msgs/Image](http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html)
* `/tello/image/raw/h264` [h264_image_transport/H264Packet](https://github.com/tilk/h264_image_transport/blob/master/msg/H264Packet.msg)
* `/tello/camera/camera_info` [sensor_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html)
* `/tello/status` [tello_driver/TelloStatus](./msg/tello_status.msg)

### Subscribe topics
* `/tello/take_off` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* `/tello/land` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* `/tello/cmd_vel` [geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)

### Services
* `/tello_node/move_up`
* `/tello_node/move_down`

### Parameters
* ```~/tello_node/tello_ip```
* ```~/tello_node/tello_cmd_server_port```
* ```~/tello_node/local_cmd_client_port```
* ```~/tello_node/local_vid_server_port```
* ```~/tello_node/initial_frame```
* ```~/tello_node/connect_time_out```
* ```~/tello_node/h264_encoder_stream```
* ```~/tello_node/camera_calibration```


## 3. Feature

### Video streaming
A raw video streaming could be used in this package stack depends on the [pyav](https://github.com/PyAV-Org/PyAV) package library. 
* `$ pip install av --user`

Note that installation based on Ubuntu 16.04 requires **ffmpeg** of at least version 3
* `$ sudo add-apt-repository ppa:jonathonf/ffmpeg-3`
* `$ sudo apt update && sudo apt install ffmpeg`

### ORB SLAM2 launch 
The drone driver could also launch the ORB SLAM2 processing node as well. By processing the video stream from tello drone to the visual SLAM algorithms. It could directly enable localization and mapping.
* `$ roslaunch tello_node tello_orb.launch`