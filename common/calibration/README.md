# calibration

## desc

根据激光雷达与摄像头的外参对点云和图像进行融合，外参可通过 autoware 激光雷达-摄像头联合标定工具得到。融合结果为点云到图像的映射（图像测距）以及图像到点云的映射（点云着色）。实际使用中摄像头和激光雷达的内参也应事先分别标定，这里仅对图像进行了矫正。

## usage

```
roslaunch calibration lidar_cam.launch
```

若外参存在问题，可通过 rqt_reconfigure 手动进行微调。

## 1121 update

增加 calibrated_camera_1121.yaml 用于 720P 的矫正。

## 1226 update

增加 calibrated_camera_1226.yaml 用于 1080P 的矫正；增加 1226_calibration.yaml 和 1226_calibration_2.yaml，保存了 lidar to camera 的旋转平移矩阵。

## ref

1. [激光雷达和相机的联合标定（Camera-LiDAR Calibration）之Autoware](https://blog.csdn.net/learning_tortosie/article/details/82347694)
2. [无人驾驶汽车系统入门（二十二）——使用Autoware实践激光雷达与摄像机组合标定](https://blog.csdn.net/AdamShan/article/details/81670732)