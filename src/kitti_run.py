#!/usr/bin/python3
import os
import time
import argparse
from data_utils import *
from publish_utils import *
from kitti_utils import *
from processor import *



DATA_PATH = '/home/brwei01/Data/data_raw/2011_09_26/2011_09_26_drive_0005_sync/'
# DATA_PATH = '/home/brwei01/Data/data_odometry/sequences/'
TRACKING_PATH = '/home/brwei01/Data/data_label/training/label_02/0000.txt'
TRACKING_PATH_YOLO_SLAM = '/home/brwei01/Dev/COMP0130_22-23_Topic_03/Coursework_03/Results/console_log_formatted.txt'
EGOCAR_BOX = np.array([[2.15, 0.9, -1.73],[2.15, -0.9, -1.73],[-1.95, -0.9,-1.73],[-1.95,0.9,-1.73],
                        [2.15, 0.9, -0.23],[2.15, -0.9, -0.23],[-1.95, -0.9,-0.23],[-1.95,0.9,-0.23],
                       ])
EGOCAR_POINT = np.array([0,0,0])


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Visualization ground-truth distance calculated by different methods")
    parser.add_argument("--dist_pcl", action="store_true",  help="Calculate distances to ego car by point cloud")
    parser.add_argument("--dist_outer", action="store_true", default=True, help="Calculate distances to ego car by bounding box")
    args = parser.parse_args()
    
    # initializations
    folder_path = os.path.join(DATA_PATH, 'image_02/data/')
    file_list = os.listdir(folder_path)
    frame_count = len([file for file in file_list if os.path.isfile(os.path.join(folder_path, file))])
    frame = 0
    dist_flag = 1
    rospy.init_node('kitti_node', anonymous=True) # create a node

    # create publishers
    cam_pub = rospy.Publisher('kitti_cam', Image, queue_size=10)
    pcl_pub = rospy.Publisher('kitti_point_cloud', PointCloud2, queue_size=10)
    imu_pub = rospy.Publisher('kitti_imu', Imu, queue_size=10)
    gps_pub = rospy.Publisher('kitti_gps', NavSatFix, queue_size=10)
    ego_pub = rospy.Publisher('ego_car', MarkerArray, queue_size=10)
    box3d_pub = rospy.Publisher('kitti_3d', MarkerArray, queue_size=10)
    dist_pub = rospy.Publisher('kitti_dist', MarkerArray, queue_size=10)
    dist_pub_pcl = rospy.Publisher('kitti_dist_pcl', MarkerArray, queue_size=10)
    
    # create bridge for datatype transformations
    bridge = CvBridge()

    # define rate
    rate = rospy.Rate(10) # 10 times per sec

    # read ground truth labels and detection results 
    df_tracking = read_tracking(TRACKING_PATH)
    df_tracking_res = read_tracking_res(TRACKING_PATH_YOLO_SLAM)

    # define calibration settings
    calib = Calibration('/home/brwei01/Data/data_raw/2011_09_26/', from_video=True)

    while not rospy.is_shutdown():
        
        # time logger
        start_time = time.time()

        # READ DATA
        # ===============================================================
        # read ground truth labels and detection results for current frame
        df_tracking_frame = df_tracking[df_tracking.frame==frame]
        df_tracking_frame_res = df_tracking_res[df_tracking_res.frame==frame]
        
        # read image data
        image = read_camera(os.path.join(DATA_PATH,'image_02/data/%010d.png'%frame)) # 10 int digits with 0 on the left

        # read the YOLO-SLAM detection result for frame viewer
        boxes_2d = matcher(df_tracking_frame, df_tracking_frame_res)
        # the YOLO-SLAM calculated distance
        distance = np.array(df_tracking_frame_res['distance']) # not used yet

        # read the Ground-Truth labels for frame viewer (not used, if used please create a separate publisher)
        # boxes_2d = np.array(df_tracking[df_tracking.frame==frame][['bbox_left','bbox_top','bbox_right','bbox_bottom']]) 

        # read the ground truth 3d labels
        boxes_3d = np.array(df_tracking_frame[['height','width','length','pos_x','pos_y','pos_z','rot_y']])
        
        # read ground truth lablling types
        types = np.array(df_tracking_frame['type'])   
        
        # read point cloud data
        point_cloud = read_point_cloud(os.path.join(DATA_PATH, 'velodyne_points/data/%010d.bin'%frame))
        
        # read imu/gps data
        imu_data = read_imu(os.path.join(DATA_PATH, 'oxts/data/%010d.txt'%frame))

        
        # This section calculates and reads distances for later publishing
        # define variables needed
        ego_car_point_2d = EGOCAR_POINT[:2]
        velo_points_2d = point_cloud[::5][:,:2]
        corners_3d_velos = []
        minPDs = [] # stores the P and distance between P and EGOCAR_POINT
        minPQDs = []  # stores the P, Q and distance between 
        # start box loop
        for box_3d in boxes_3d:
            corners_3d_cam2 = compute_3d_box_cam2(*box_3d) # calcualte the corners
            corners_3d_velo = calib.project_rect_to_velo(corners_3d_cam2.T) # convert to velo, 8*3 array
            corners_3d_velos += [corners_3d_velo]

            if args.dist_outer:
                minPQDs += [min_distance_cuboids(EGOCAR_BOX, corners_3d_velo)]
                dist_flag = 1 # linked to running time
            if args.dist_pcl:
                # judge if point in box_3d
                box_2d_pcl = Polygon(corners_3d_velo[:5,:2])
                minPDs += [min_distance_2dpip(ego_car_point_2d, velo_points_2d, box_2d_pcl)]
                dist_flag = 2 # linked to running time
        corners_3d_velos += [EGOCAR_BOX]
        types = np.append(types, 'Car')
        # =================================================================
        # SECTION ENDS
            


        # PUBLISH SECTION
        # ==================================================================
        # publish data 
        # publish_camera(cam_pub, bridge, image, boxes_2d, types)
        if frame in df_tracking.frame:
            d_type = 'Car' # harde coded, modify SLAM if needed
            publish_camera_res(cam_pub, bridge, image, boxes_2d, distance)
        else:
            publish_camera_no_detection(cam_pub, bridge, image)
        publish_point_cloud(pcl_pub, point_cloud)
        publish_imu(imu_pub, imu_data)
        publish_gps(gps_pub, imu_data)
        # publish visualizations
        publish_ego_car(ego_pub)
        publish_3dbox(box3d_pub, corners_3d_velos, types, dist_flag)

        if dist_flag == 1:
            publish_dist(dist_pub, minPQDs)
        if dist_flag == 2:
            publish_dist_pcl(dist_pub_pcl, EGOCAR_POINT ,minPDs)
        # ===================================================================
        # SECTION ENDS


        # log time
        end_time = time.time()  # Record the end time
        elapsed_time = end_time - start_time  # Calculate the elapsed time for this iteration


        # log info
        rospy.loginfo("published frame %d"%frame)
        rospy.loginfo("Published frame %d in %.3f seconds" % (frame, elapsed_time))
        rate.sleep()

        # loop the programme
        frame += 1
        frame %= frame_count # the largest frame series num is frame_count, if it exceeds, go back to 0


















