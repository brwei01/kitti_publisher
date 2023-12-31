#!/usr/bin/python3

import rospy
import cv2
import tf
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Image, PointCloud2, Imu, NavSatFix
from geometry_msgs.msg import Point
import sensor_msgs.point_cloud2 as pcl2
from std_msgs.msg import Header
from cv_bridge import CvBridge


FRAME_ID = 'map'
DETECTION_COLOR_DICT = {'Car':(255,255,0),'Pedestrian':(0,226,255),'Cyclist':(141,40,255)}
LIFE_TIME = 0.1 # the camera fps=10
LIFE_TIME_PIP = 1.1 # avg processing time for pip is 1.1s
LINES = [[0,1],[1,2],[2,3],[3,0]]
LINES += [[4,5],[5,6],[6,7],[7,4]]
LINES += [[4,0],[5,1],[6,2],[7,3]]
LINES += [[4,1],[5,0]] # front


def publish_camera(cam_pub, bridge, image, boxes, types):
    # for dist, box in zip(boxes, distance):
    for type, box in zip(types, boxes):
        pt1 = (int(box[0]),int(box[1]))
        pt2 = (int(box[2]),int(box[3]))
        cv2.rectangle(image, pt1, pt2, DETECTION_COLOR_DICT[type], 2)
    cam_pub.publish(bridge.cv2_to_imgmsg(image, "bgr8"))

def publish_camera_res(cam_pub, bridge, image, boxes, distance):
    # for dist, box in zip(boxes, distance):
    for dist, box in zip(distance, boxes):
        pt1 = (int(box[0]),int(box[1]))
        pt2 = (int(box[2]),int(box[3]))
        cv2.rectangle(image, pt1, pt2, DETECTION_COLOR_DICT['Car'], 2)

        # Add the distance parameter as text annotation
        text = f"{dist:.2f}"  # Format the distance with two decimal places
        text_position = (int(box[0]), int(box[1]) - 10)  # Position above the bounding box
        font = cv2.LINE_AA
        font_scale = 1.0
        font_color = (0, 0, 255)  # White color
        font_thickness = 2
        
        cv2.putText(image, text, text_position, font, font_scale, font_color, font_thickness)

    cam_pub.publish(bridge.cv2_to_imgmsg(image, "bgr8"))


def publish_camera_no_detection(cam_pub, bridge, image):
    cam_pub.publish(bridge.cv2_to_imgmsg(image, "bgr8"))

def publish_point_cloud(pcl_pub, point_cloud):
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = FRAME_ID
    pcl_pub.publish(pcl2.create_cloud_xyz32(header, point_cloud[:,:3]))

def publish_imu(imu_pub, imu_data):
    # Ref
    # http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html
    imu = Imu()
    imu.header.frame_id = FRAME_ID
    imu.header.stamp = rospy.Time.now()

    # set rotation
    q = tf.transformations.quaternion_from_euler(
        float(imu_data.roll.iloc[0]),
        float(imu_data.pitch.iloc[0]),
        float(imu_data.yaw.iloc[0])
    )
    imu.orientation.x = q[0]
    imu.orientation.y = q[1]
    imu.orientation.z = q[2]
    imu.orientation.w = q[3]

    # acceleration
    imu.linear_acceleration.x = float(imu_data.af.iloc[0])
    imu.linear_acceleration.y = float(imu_data.al.iloc[0])
    imu.linear_acceleration.z = float(imu_data.au.iloc[0])
    
    # angular velocity    
    imu.angular_velocity.x = float(imu_data.wf.iloc[0])
    imu.angular_velocity.y = float(imu_data.wl.iloc[0])
    imu.angular_velocity.z = float(imu_data.wu.iloc[0])

    imu_pub.publish(imu)

def publish_gps(gps_pub, imu_data):
    gps = NavSatFix()
    gps.header.frame_id = FRAME_ID
    gps.header.stamp = rospy.Time.now()

    gps.latitude = imu_data.lat
    gps.longitude = imu_data.lon
    gps.altitude = imu_data.alt

    gps_pub.publish(gps)

def publish_ego_car(ego_car_pub):
    """
    publish left and right 45 degree FOV lines and ego car model mesh
    """
    marker_array = MarkerArray() # for simultaneous publish

    marker = Marker()
    marker.header.frame_id = FRAME_ID
    marker.header.stamp = rospy.Time.now()

    marker.id = 0
    marker.action = Marker.ADD
    marker.lifetime = rospy.Duration() # this shows the marker the whole span while the programme runs
    marker.type = Marker.LINE_STRIP

    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0 # alpha, transparency
    marker.scale.x = 0.2

    marker.points = []
    marker.points.append(Point(10,-10,0))
    marker.points.append(Point(0,0,0))
    marker.points.append(Point(10,10,0))

    marker_array.markers.append(marker)

    # car model
    mesh_marker = Marker()
    mesh_marker.header.frame_id = FRAME_ID
    mesh_marker.header.stamp = rospy.Time.now()

    mesh_marker.id = -1
    mesh_marker.lifetime = rospy.Duration()
    mesh_marker.type = Marker.MESH_RESOURCE
    mesh_marker.mesh_resource = "package://kitti_publisher/Car_Model/Car-Model/Car.dae"

    mesh_marker.pose.position.x = 0.0
    mesh_marker.pose.position.y = 0.0
    mesh_marker.pose.position.z = -1.73

    q = tf.transformations.quaternion_from_euler(0, 0, np.pi/2)
    mesh_marker.pose.orientation.x = q[0]
    mesh_marker.pose.orientation.y = q[1]
    mesh_marker.pose.orientation.z = q[2]
    mesh_marker.pose.orientation.w= q[3]

    mesh_marker.color.r = 1.0
    mesh_marker.color.g = 1.0
    mesh_marker.color.b = 1.0
    mesh_marker.color.a = 1.0

    mesh_marker.scale.x = 0.8
    mesh_marker.scale.y = 0.8
    mesh_marker.scale.z = 0.8

    marker_array.markers.append(mesh_marker)

    ego_car_pub.publish(marker_array)

def publish_car_model(model_pub):
    '''
    this has to be downloaded from online resources (.dae)
    put the .dae under src folder
    mesh_marker.mesh_resource = 'package://package_name/path/to/file.dae'
    3d mesh can be pusblished by
    MESH_RESOURCE=10
    this will be updated later
    '''
    return

def publish_3dbox(box3d_pub, corners_3d_velos, obj_types, dist_flag):
    marker_array = MarkerArray()
    for i, corners_3d_velo in enumerate(corners_3d_velos):
        marker = Marker()
        marker.header.frame_id = FRAME_ID
        marker.header.stamp = rospy.Time.now()

        marker.id = i
        marker.action = Marker.ADD
        if dist_flag == 1:
            marker.lifetime = rospy.Duration(LIFE_TIME)
        elif dist_flag == 2:
            marker.lifetime = rospy.Duration(LIFE_TIME_PIP)
        marker.type = Marker.LINE_LIST
        
        b, g, r = DETECTION_COLOR_DICT[obj_types[i]]
        marker.color.r = r/255.0
        marker.color.g = g/255.0
        marker.color.b = b/255.0
        
        marker.color.a = 1.0
        marker.scale.x = 0.1

        marker.points = []
        for l in LINES:
            p1 = corners_3d_velo[l[0]]
            marker.points.append(Point(p1[0], p1[1], p1[2]))
            p2 = corners_3d_velo[l[1]]
            marker.points.append(Point(p2[0], p2[1], p2[2])) 
        marker_array.markers.append(marker)

    box3d_pub.publish(marker_array)       

def publish_dist(dist_pub, minPQDs):
    marker_array = MarkerArray()

    for i, (minP, minQ, minD) in enumerate(minPQDs):
        marker = Marker()
        marker.header.frame_id = FRAME_ID
        marker.header.stamp = rospy.Time.now()

        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration(LIFE_TIME)
        marker.type = Marker.LINE_STRIP
        marker.id = i 

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 0.5
        marker.scale.x = 0.1

        marker.points = []
        marker.points.append(Point(minP[0], minP[1], 0))
        marker.points.append(Point(minQ[0], minQ[1], 0))

        marker_array.markers.append(marker)

        text_marker = Marker()
        text_marker.header.frame_id = FRAME_ID
        text_marker.header.stamp = rospy.Time.now()

        text_marker.id = i + 1000
        text_marker.action = Marker.ADD
        text_marker.lifetime = rospy.Duration(LIFE_TIME)
        text_marker.type = Marker.TEXT_VIEW_FACING

        p = (minP + minQ) / 2.0
        text_marker.pose.position.x = p[0]
        text_marker.pose.position.y = p[1]
        text_marker.text = '%.2f'%minD

        text_marker.scale.x = 1
        text_marker.scale.y = 1
        text_marker.scale.z = 1

        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 0.8
        marker_array.markers.append(text_marker)

    dist_pub.publish(marker_array)

def publish_dist_pcl(dist_pub_pcl, EGOCAR_POINT, minPDs):
    marker_array = MarkerArray()

    for i, (minP, minD) in enumerate(minPDs):
        marker = Marker()
        marker.header.frame_id = FRAME_ID
        marker.header.stamp = rospy.Time.now()

        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration(LIFE_TIME_PIP)
        marker.type = Marker.LINE_STRIP
        marker.id = i 

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 0.5
        marker.scale.x = 0.1

        marker.points = []
        marker.points.append(Point(minP[0], minP[1], 0))
        marker.points.append(Point(EGOCAR_POINT[0], EGOCAR_POINT[1], 0))

        marker_array.markers.append(marker)

        text_marker = Marker()
        text_marker.header.frame_id = FRAME_ID
        text_marker.header.stamp = rospy.Time.now()

        text_marker.id = i + 1000
        text_marker.action = Marker.ADD
        text_marker.lifetime = rospy.Duration(LIFE_TIME_PIP)
        text_marker.type = Marker.TEXT_VIEW_FACING

        p = (minP + EGOCAR_POINT) / 2.0
        text_marker.pose.position.x = p[0]
        text_marker.pose.position.y = p[1]
        text_marker.text = '%.2f'%minD

        text_marker.scale.x = 1
        text_marker.scale.y = 1
        text_marker.scale.z = 1

        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 0.8
        marker_array.markers.append(text_marker)

    dist_pub_pcl.publish(marker_array)