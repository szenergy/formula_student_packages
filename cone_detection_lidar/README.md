# `cone_detection_lidar` ROS 2 package

The code is based on https://github.com/jkk-research/lidar_cluster_ros2

[![Static Badge](https://img.shields.io/badge/ROS_2-Humble-34aec5)](https://docs.ros.org/en/humble/)


## Build
<details>
<summary> colcon build </summary>

``` bash
cd ~/ros2_ws/ && colcon build --symlink-install --packages-select cone_detection_lidar
```
</details>


## Remarks

In VS code it is advised to add the following to include path:

``` r
${workspaceFolder}/**
/opt/ros/humble/include/**
/usr/include/pcl-1.12/**
/usr/include/eigen3/**
```

If you are not sure where your header files are use e.g.:
``` r
find /usr/include -name point_cloud.h
find /usr/include -name crop_box.h
```