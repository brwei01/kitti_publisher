#!/usr/bin/python3
import os
from data_utils import *
from publish_utils import *

# DATA_PATH = '/home/brwei01/Data/data_raw/2011_09_26/2011_09_26_drive_0005_sync/'
DATA_PATH = '/home/brwei01/Data/data_odometry/sequences/'
TRACKING_PATH = '/home/brwei01/Dev/COMP0130_22-23_Topic_03/Coursework_03/Results/console_log_formatted.txt'

if __name__ == "__main__":
    frame = 0
    rospy.init_node('kitti_node', anonymous=True) # create a node
    cam_pub = rospy.Publisher('kitti_cam', Image, queue_size=10)
    pcl_pub = rospy.Publisher('kitti_point_cloud', PointCloud2, queue_size=10)
    ego_pub = rospy.Publisher('ego_car', Marker, queue_size=10)
    bridge = CvBridge()


    rate = rospy.Rate(10) # 10 times per sec

    df_tracking = read_tracking(TRACKING_PATH)

    while not rospy.is_shutdown():
        image = read_camera(os.path.join(DATA_PATH,'03/image_0/%06d.png'%frame)) # 10 int digits with 0 on the left
        if frame in df_tracking.frame:
            boxes = np.array(df_tracking[df_tracking.frame==frame][['left','top','right','bottom']])
            distance = np.array(df_tracking[df_tracking.frame==frame]['distance'])
            publish_camera(cam_pub, bridge, image, boxes, distance)
        else:
            publish_camera_0(cam_pub, bridge, image)
            
        point_cloud = read_point_cloud(os.path.join(DATA_PATH, '03_velodyne/%06d.bin'%frame))
        publish_point_cloud(pcl_pub, point_cloud)
        publish_ego_car(ego_pub)

        rospy.loginfo("camera image and velodyne point cloud published")
        rate.sleep()
        frame += 1
        frame %= 801 # the largest frame series num is 153, if it exceeds, go back to 0


