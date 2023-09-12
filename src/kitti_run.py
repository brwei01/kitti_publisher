#!/usr/bin/python3
import os
import time
import argparse
from data_utils import *
from publish_utils import *
from kitti_utils import *
from processor import *



DATA_PATH = '/home/brwei01/Data/data_tracking/sequences/0002/'
IMU_PATH = '/home/brwei01/Data/data_tracking/oxts/0002.txt'
# DATA_PATH = '/home/brwei01/Data/data_odometry/sequences/'
TRACKING_PATH = '/home/brwei01/Data/data_tracking/labels/0002.txt'
TRACKING_PATH_YOLO_SLAM = '/home/brwei01/Dev/SLAM_OFFLINE/COMP0130_22-23_Topic_03/Coursework_03/Results/0002/console_log_formatted_0002.txt'

CALIB_FILE_PATH = '/home/brwei01/Data/data_tracking/calibration/0002.txt'
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
    folder_path = os.path.join(DATA_PATH, 'image_0/')
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
    calib = Calibration(CALIB_FILE_PATH, from_video=True)

    # ready to save file
    save_file = True
    # Create list to store data for all frames
    all_frame_data = []

    while not rospy.is_shutdown():
        
        # time logger
        start_time = time.time()

        # READ DATA
        # ===============================================================
        # read ground truth labels and detection results for current frame
        df_tracking_frame = df_tracking[df_tracking.frame==frame]
        df_tracking_frame_res = df_tracking_res[df_tracking_res.frame==frame]
        
        # read image data
        image = read_camera(os.path.join(DATA_PATH,'image_0/%06d.png'%frame)) # 10 int digits with 0 on the left

        # read the YOLO-SLAM detection result for frame viewer
        # boxes_2d = matcher(df_tracking_frame, df_tracking_frame_res)
        boxes_2d = get_boxes_2d(df_tracking_frame_res)

        # read the YOLO-SLAM calculated distance
        distance_res = np.array(df_tracking_frame_res['distance']) # not used yet

        # read the Ground-Truth labels for frame viewer (not used, if used please create a separate publisher)
        # boxes_2d = np.array(df_tracking[df_tracking.frame==frame][['bbox_left','bbox_top','bbox_right','bbox_bottom']]) 

        # read the ground truth 3d labels
        boxes_3d = np.array(df_tracking_frame[['height','width','length','pos_x','pos_y','pos_z','rot_y']])
 
        
        # read ground truth lablling types
        types = np.array(df_tracking_frame['type'])   
        
        # read point cloud data
        point_cloud = read_point_cloud(os.path.join(DATA_PATH, 'velodyne_points/%06d.bin'%frame))
        
        # read imu/gps data
        # imu_data = read_imu(os.path.join(IMU_PATH, 'oxts/data/%06d.txt'%frame)) # this works for raw data
        imu_data = read_imu_txt(IMU_PATH, frame) # this works for tracking benchmark with all imu readings in one file

        
        # This section calculates and reads distances for later publishing
        # define variables needed
        ego_car_point_2d = EGOCAR_POINT[:2]
        velo_points_2d = point_cloud[::5][:,:2]
        corners_3d_velos = []
        minPDs = [] # stores the P and distance between P and EGOCAR_POINT
        minPQDs = []  # stores the P, Q and distance between 
        min_distances = [] # store the min distance calculated from ground truth
        max_distances = []
        mean_distances = []
        
        # start box loop
        for i, box_3d in enumerate(boxes_3d):
            # read the ground truth centroids
            centroid_3d = np.array(box_3d[[3,4,5]]).reshape(3,1) # extract centroid from 3d bbox
            centroid_3d_velo = calib.project_rect_to_velo(centroid_3d.T) # 1*3 
            
            corners_3d_cam2 = compute_3d_box_cam2(*box_3d) # calcualte the corners
            corners_3d_velo = calib.project_rect_to_velo(corners_3d_cam2.T) # convert to velo, 8*3 array
            corners_3d_velos += [corners_3d_velo]
            
        

            if args.dist_outer:
                minP, minQ, minD = min_distance_cuboids(EGOCAR_BOX, corners_3d_velo)
                maxD = np.nan # TODO
                meanD = np.nan # TODO
                minPQDs += [(minP, minQ, minD)]
                dist_flag = 1 # linked to running time
            if args.dist_pcl:
                # judge if point in box_3d
                box_2d_pcl = Polygon(corners_3d_velo[:5,:2])
                minP, minD, maxD = min_max_distance_2dpip(ego_car_point_2d, velo_points_2d, box_2d_pcl)
                meanD = mean_distance(EGOCAR_POINT, centroid_3d_velo)
                minPDs += [(minP, minD)]
                dist_flag = 2 # linked to running time
                # only valid for pcl distance
                
            max_distances.append(maxD)    
            mean_distances.append(meanD)
            min_distances.append(minD)
            
                
        
        corners_3d_velos += [EGOCAR_BOX]
        types = np.append(types, 'Car')

        # Saving data to file
        # Create a new DataFrame to store the data including 'minD'
        if save_file:
            output_df = pd.DataFrame({
                'frame': df_tracking_frame['frame'],
                'track_id': df_tracking_frame['track_id'],
                'bbox_left': df_tracking_frame['bbox_left'],
                'bbox_top': df_tracking_frame['bbox_top'],
                'bbox_right': df_tracking_frame['bbox_right'],
                'bbox_bottom': df_tracking_frame['bbox_bottom'],
                'distance_gt_min': min_distances,
                'distance_gt_mean': mean_distances,
                'distance_gt_max': max_distances
            })
            all_frame_data.append(output_df)
            print("saving data")
        

        # =================================================================
        # SECTION ENDS
            


        # PUBLISH SECTION
        # ==================================================================
        # publish data 
        # publish_camera(cam_pub, bridge, image, boxes_2d, types)
        if frame in df_tracking.frame.values:
            detect_type = 'Car' # hard coded, modify SLAM if needed
            publish_camera_res(cam_pub, bridge, image, boxes_2d, distance_res)
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
        if frame > frame_count-1:
            # Create a DataFrame from the all_frame_data list
            all_frames_df = pd.concat(all_frame_data, ignore_index=True)
            filename = DATA_PATH.split('/')[-2]
            output_file_path = f'results/{filename}.txt'
            # Write the DataFrame to a CSV file
            all_frames_df.to_csv(output_file_path, index=False)
            save_file = False
            frame %= frame_count # the largest frame series num is frame_count, if it exceeds, go back to 0



















