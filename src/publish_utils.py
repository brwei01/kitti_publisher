#!/usr/bin/python3

import rospy
import cv2
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Point
import sensor_msgs.point_cloud2 as pcl2
from std_msgs.msg import Header
from cv_bridge import CvBridge


FRAME_ID = 'map'
DETECTION_COLOR_DICT = {'Car':(255,255,0),'Pedestrain':(0,226,255),'Cyclist':(141,40,255)}

def publish_camera(cam_pub, bridge, image, boxes, distance):
    # for dist, box in zip(boxes, distance):
    for box in boxes:
        pt1 = int(box[0]), int(box[1])
        pt2 = int(box[2]), int(box[3])
        cv2.rectangle(image, pt1, pt2, DETECTION_COLOR_DICT['Car'], 2)
    cam_pub.publish(bridge.cv2_to_imgmsg(image, "bgr8"))

def publish_camera_0(cam_pub, bridge, image):
    cam_pub.publish(bridge.cv2_to_imgmsg(image, "bgr8"))


def publish_point_cloud(pcl_pub, point_cloud):
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = FRAME_ID
    pcl_pub.publish(pcl2.create_cloud_xyz32(header, point_cloud[:,:3]))

def publish_ego_car(ego_car_pub):
    """
    publish left and right 45 degree FOV lines and ego car model mesh
    """
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

    ego_car_pub.publish(marker)

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