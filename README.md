# kitti_publisher

this repository is used for validation.

## Prerequisites:
Ubuntu 20.04
ROS Noetic 1.16.0
Rviz

## Build project
clone to ~/catkin_ws/src or customed ROS working directory
'''
cd ~/catkin_ws/src
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
cd ..
catkin_make
'''

## Run the project
in terminal 1:
'''
$ roscore
'''

in termnal 2:
'''
$ cd catkin_ws
$ source /opt/ros/noetic/setup.bash
$ source ~/catkin_ws/devel/setup.bash
$ cd src
$ roscd kitti_publisher/src
$ chmod +x kitti_run.py
$ rosrun kitti_publisher kitti_run.py --dist_pcl
'''

in terminal 3:
'''
$ cd catkin_ws
$ source /opt/ros/noetic/setup.bash
$ source ~/catkin_ws/devel/setup.bash
$ rviz
'''
