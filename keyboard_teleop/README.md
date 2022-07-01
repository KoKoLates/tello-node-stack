# Keyborad Teleop
Keyborad teleop for DJI Tello for ROS noetic. This keyborad teleop is derivative from [teleop twist keyborad](https://github.com/ros-teleop/teleop_twist_keyboard) of ros teleop, and add to the `takeoff` and `land` function for the drone.

## Key Bindings
**LEFT HAND:** <BR>
* `w` : Up
* `s` : Down
* `a` : Yaw Left
* `d` : Yaw Right

**RIGHT HAND:** <BR>
* `i` : Forward
* `k` : Backward
* `j` : Left
* `l` : Right

**TRIGGER:** <BR>
* `+` : Take Off
* `-` : Land
* `Else Key` : Stop

## Repeat Rate
If your mobile base requires constant updates on the `cmd_vel` topic, keyboard teleop can be configured to repeat the last command at a fixed interval, using the `repeat_rate` private parameter.

For example, to repeat the last command at 10Hz:
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py _repeat_rate:=10.0
```
It is highly recommened that the repeat rate be used in conjunction with the key timeout, to prevent runaway robots.

## Key Timeout
Keyboard teleop can be configured to stop your robot if it does not receive any key presses in a configured time period, using the `key_timeout` private parameter.

For example, to stop your robot if a keypress has not been received in 0.6 seconds:
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py _key_timeout:=0.6
```
It is recommended that you set `key_timeout` higher than the initial key repeat delay on your system (This delay is 0.5 seconds by default on Ubuntu, but can be adjusted ).