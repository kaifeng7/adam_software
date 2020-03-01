# gen_costmap2

接收全局静态栅格地图、实时点云观测数据以及激光雷达的实时位置（在全局静态栅格地图中的位置）来对地图进行更新，仅对最近的观测障碍物到激光雷达之间的栅格进行更新。

相当于将障碍物点云投影成一个 fake 单线雷达 scan，然后根据各个角度最近的障碍物对地图进行更新。

## 改进计划

算法主要耗时在 raycasting 部分，目前使用最基础的 Bresenham's line 算法进行 raycasting，后续可以采用效率更高的算法，如 MIT Corey H.等人提出的 CDDT 方法（该方法在 pf_localization 中使用），或使用 OpenMP/CUDA 进行并行化，但也要注意 TX2 上资源的使用。

为栅格维护状态，采用卡尔曼滤波进行更新，减少噪声的影响。

## ref

1. [Stanford DARPA Junior]
2. [MIT CDDT](https://arxiv.org/pdf/1705.01167.pdf)