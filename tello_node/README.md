# Drone Driver Node
The `DJI TELLO` driver node on ROS noetic based on unofficial [TelloPy](https://github.com/hanyazou/TelloPy) library.

## 1. Node
### Publish topics
* `/tello/image_raw` [sensor_msgs/Image](http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html)
* `/tello/image/raw/h264` [h264_image_transport/H264Packet](https://github.com/tilk/h264_image_transport/blob/master/msg/H264Packet.msg)
* `/tello/camera/camera_info` [sensor_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html)
* `/tello/status` [tello_driver/TelloStatus](./msg/tello_status.msg)

### Subscribe topics
* `/tello/take_off` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* `/tello/land` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* `/tello/cmd_vel` [geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)

### Parameters
* ```~/tello_driver_node/connect_time_out```
* ```~/tello_driver_node/local_cmd_client_port```
* ```~/tello_driver_node/local_vid_server_port```
* ```~/tello_driver_node/stream_h264_video```
* ```~/tello_driver_node/tello_cmd_server_port```
* ```~/tello_driver_node/tello_ip```
* ```~/tello_driver_node/initial_frame```
* ```~/tello_driver_node/camera_calibration```
