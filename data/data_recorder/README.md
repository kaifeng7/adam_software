# data_recorder

## usage

```shell
roslaunch data_recorder record_outdoor_1217.launch

rosbag record --output-name=outdoor_1217_all --buffsize=0 /cv_cam/image_raw/compressed /cv_cam/camera_info /tf /tf_static  /lslidar_point_cloud /hall_sensor /wheel_circles /imu/data /imu/data_raw /odom/imu /raw_imu
```