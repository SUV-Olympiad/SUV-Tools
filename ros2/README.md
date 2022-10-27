# ROS2

### version
- ROS 2 Foxy Fitzroy


## Build

- In `SUV-Tools/ros2`


- `colcon build --symlink-install`


- `source ./install/setup.bash`


## px4_msgs

- PX4의 uorb 메시지를 rtps msg 타입으로 변환
- PX4-Autopilot [msg](https://github.com/SUV-Olympiad/PX4-Autopilot/tree/olympiad/msg)와 동일하게 유지 되어야 한다


## leapmotion

- LeapMotion을 이용하여 손의 `roll`, `pitch`, `yaw`, `height`, `grip` 전송
- LeapMotion 데이터에 맞는 [Leap.msg](./src/suv_msgs/msg/Leap.msg) 정의
- `python3 LeapControl.py` 로 실행


## rtps_command

- ROS2를 이용하여 PX4에게 Command 명령하는 [example code](./src/rtps_command/rtps_command/test2.py)
- sensor_msgs 타입의 Image [영상 출력](./src/rtps_command/rtps_command/camera.py)