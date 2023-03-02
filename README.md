# imu_xg6000_ros2
ROS2 package for CruzeCore Microinfinity XG6000 

## Usage

```
$ ros2 run imu_xg6000_ros2 main_node --ros-args -p port_name:=/dev/ttyUSB0 -p baudrate:=38400 -p frame_id:=imu_link
```

## Topics

- imu_raw [sensor_msgs/msg/Imu]
