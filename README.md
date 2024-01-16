# Formula Student Driverless - developed by University of Gyor

## Proposed system architecture
The proposed system architecture is shown below. The following considerations are made for each component:
- state machine: base logic of szenergy bringup is taken over, with slight modifications to FS rules; output topic includes both state and mission ID, and mission ID is set manually (external source or parameter in accordance with FS rules)
- object detection: two parallel solutions are implemented, later either of these or the combination (fusion) is used, to be decided later,
- object fusion: if e.g., only lidar detection is used, fusion is unnecessary and can be deleted,
- map creation includes SLAM
- trajectory planner receives the raw object list and plans path based on it (e.g., first round of trackdriver, acceleration) and also received the map from the slam, where global trajectory is planned; this switching is handled inside the planner,
- control in the first place is a simple take-over of the szenergy solution, later it is planned to be changed to handle high speed handling.

All components are to be developed in ROS2.

![Architecture](arch.png)
