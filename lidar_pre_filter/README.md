# `lidar_pre_filter` ROS 2 package 

[![Static Badge](https://img.shields.io/badge/ROS_2-Humble-34aec5)](https://docs.ros.org/en/humble/)

## Parameters

| Parameter             | Value e.g.     | Description                                 |
|-----------------------|----------------|---------------------------------------------|
| `cloud_in_topic`      | `/cloud_topic` | Pointcloud to process                       |
| `cam_cones_topic`     | `/cones_topic` | Coordinates of cones (from camera)          |
| `output_frame`        | `/out_frame`   | Output frame to publish (and transform) to  |
| `verbose1`            | `true`         | More log info                               |
| `verbose2`            | `true`         | More log info                               |
| `toggle_boundary_trim`| `true`         | Turn filtering ("trim") of far points ON    |
| `toggle_box_filter`   | `true`         | Turn cutting out boxes by given array ON    |
| `toggle_cam_filter`   | `true`         | Turn filtering by distance from cones ON    |
| `crop_boundary`       | 6x1 array      | Region of interest                          |
| -> [0]                | `-220.0`       | minX - no points with smaller X remain      |
| -> [1]                | `-220.0`       | minY - no points with smaller Y remain      |
| -> [2]                | `-10.0`        | minZ - no points with smaller Z remain      | 
| -> [3]                | `220.0`        | maxX - no points with greater X remain      |
| -> [4]                | `220.0`        | maxY - no points with greater Y remain      |
| -> [5]                | `5.0`          | maxZ - no points with greater Z remain      |
| `crop_box_array`      | 6xN array      | same as crop_boundary, but inverted... ->   |
| ... **see below**     | (...)          | -> ...the points WITHIN get filtered out    |

#### **crop_box_array**
| #  | minX     | minY     | minZ     | maxX     | maxY     | maxZ     | description  |
|----|----------|----------|----------|----------|----------|----------|--------------|
| 1. | `0.25,`  | `-0.35,` | `-0.25,` | `2.35,`  | `0.35,`  | `0.45,`  | main body    |
| 2. | `0.5,`   | `-0.6,`  | `-0.25,` | `1.2,`   | `0.6,`   | `0.2,`   | side parts   |
| 3. | `-0.25,` | `-0.7,`  | `-0.25,` | `0.25,`  | `0.7,`   | `0.25,`  | rear wheels  |
| 4. | `1.2,`   | `-0.8,`  | `-0.25,` | `1.8,`   | ` 0.8,`  | `0.25,`  | front wheels |
| 5. | `0.15,`  | `-0.3,`  | `0.4,`   | `0.6,`   | `0.3,`   | `1.0,`   | seat         |
| 6. | `0.5,`   | `-0.25,` | `0.4,`   | `1.35,`  | `0.25,`  | `1.0,`   | pilot        |

![Architecture](../img/filter01.png)


## Run

``` bash
ros2 launch lidar_pre_filter filter_vehicle01.launch.py topic:=/lexus3/os_center/points
```