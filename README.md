# kitti_publisher
This repository is used for validate distances estimated by ORB-SLAM2. <br>
This application provides visualization <br>
ground truth distance will be saved after running first iteration of all frames. <br>
2 ways calculating ground truth data: <br>
- minimum distance from outer bounding boxes;
- return closest point from point clouds within bounding box

![Screenshot from 2023-08-23 11-12-18](https://github.com/brwei01/kitti_publisher/assets/116358733/ab78cb5b-4f61-4d70-8bac-2794985cae36)


## Prerequisites:
Ubuntu 20.04 <br>
ROS Noetic 1.16.0 <br>
Rviz

## Build project
clone to ~/catkin_ws/src or customed ROS working directory
```
$ cd ~/catkin_ws/src
$ source /opt/ros/noetic/setup.bash
$ source ~/catkin_ws/devel/setup.bash
$ cd ..
$ catkin_make
```

## Run the project
in terminal 1:
```
$ roscore
```

in termnal 2:
```
$ cd catkin_ws
$ source /opt/ros/noetic/setup.bash
$ source ~/catkin_ws/devel/setup.bash
$ cd src
$ roscd kitti_publisher/src
$ chmod +x kitti_run.py
$ rosrun kitti_publisher kitti_run.py --dist_pcl
```

in terminal 3:
```
$ cd catkin_ws
$ source /opt/ros/noetic/setup.bash
$ source ~/catkin_ws/devel/setup.bash
$ rviz
```

## References
kitti_run.py, data_utils.py, publish.utils adapted from https://www.youtube.com/watch?v=TBdcwwr5Wyk&list=PLDV2CyUo4q-L4YlXUWDytZPz9a8cAW <br>
kitti_utils.py for calibration functions, code from https://github.com/charlesq34/frustum-pointnets/blob/master/kitti/kitti_util.py <br>
function compute_3d_boxes_cam2(), used for computing 3d bounding boxes from tracklets. code from https://github.com/bostondiditeam/kitti/blob/master/resources/devkit_object/readme.txt<br>

