#!/usr/bin/python3

import numpy as np
import pandas as pd
import shapely
from shapely import Point, Polygon

def compute_3d_box_cam2(h,w,l,x,y,z,yaw):
    '''
    Return: 3xn in cam2 coordinates
    code from: https://github.com/bostondiditeam/kitti/blob/master/resources/devkit_object/readme.txt
    '''
    R = np.array([[np.cos(yaw),0,np.sin(yaw)], [0,1,0], [-np.sin(yaw),0,np.cos(yaw)]])
    x_corners = [l/2,l/2,-l/2,-l/2,l/2,l/2,-l/2,-l/2]
    y_corners = [0,0,0,0,-h,-h,-h,-h]
    z_corners = [w/2,-w/2,-w/2,w/2,w/2,-w/2,-w/2,w/2]
    corners_3d_cam2 = np.dot(R, np.vstack([x_corners, y_corners, z_corners]))
    corners_3d_cam2 += np.vstack([x,y,z])
    return corners_3d_cam2


def distance_point_to_segment(P,A,B):
    '''
    Calculates the min distance of a point to a segment AB
    Returns the point Q in AB on which the min distance can be found
    adapted from https://www.youtube.com/watch?v=TBdcwwr5Wyk&list=PLDV2CyUo4q-L4YlXUWDytZPz9a8cAW
    '''
    AP = P-A
    BP = P-B
    AB = B-A
    if np.dot(AB, AP) >= 0 and np.dot(-AB, BP) >= 0:
        return np.abs(np.cross(AP,AB))/np.linalg.norm(AB), np.dot(AP,AB)/np.dot(AB,AB)*AB + A
    d_PA = np.linalg.norm(AP)
    d_PB = np.linalg.norm(BP)
    if d_PA < d_PB:
        return d_PA, A
    return d_PB, B

def min_distance_cuboids(cub1, cub2):
    '''
    Computes the minimum distance between two non-overlapping cuboids (3D) of shape(8,3)
    They are projected to BEV and the minimum distance of the two rectangles are return.
    adapted from https://www.youtube.com/watch?v=TBdcwwr5Wyk&list=PLDV2CyUo4q-L4YlXUWDytZPz9a8cAW
    '''
    minD = 1e5
    for i in range(4):
        for j in range(4):
            d, Q = distance_point_to_segment(cub1[i,:2], cub2[j,:2], cub2[j+1,:2])
            if d < minD:
                minD = d
                minP = cub1[i, :2]
                minQ = Q
    for i in range(4):
        for j in range(4):
            d, Q = distance_point_to_segment(cub2[i,:2], cub1[j,:2], cub1[j+1,:2])
            if d < minD:
                minD = d
                minP = cub2[i, :2]
                minQ = Q
    
    return minP, minQ, minD

def mean_distance(ego_car_3d, object_centroid):
    meanD = np.linalg.norm(object_centroid - ego_car_3d)
    print(meanD)
    return meanD


def min_max_distance_2dpip(ego_car_point_2d, velo_points_2d, box_2d_pcl):
    '''
    Computes the distance between camera point(0,0,0)
    and the closest and farthest laser points in 2d within a 2d object bounding box
    '''
    points_in_box = []
    for point_coords in velo_points_2d:
        point = Point(point_coords)
        if point.within(box_2d_pcl):
            points_in_box.append(point_coords)
    
    if len(points_in_box) > 0:
        distances = np.linalg.norm(points_in_box - ego_car_point_2d, axis=1)
        index_of_shortest_distance = np.argmin(distances)
        index_of_longest_distance = np.argmax(distances)
        
        minD = distances[index_of_shortest_distance]
        maxD = distances[index_of_longest_distance]
        
        minP = points_in_box[index_of_shortest_distance]
        minP = np.concatenate((minP, np.array([0])))
        
    else: 
        minD = -1
        maxD = -1
        minP = np.array([0.,0.,0.])

    return minP, minD, maxD



def matcher(df_tracking_frame_gt, df_tracking_frame_res):
    # turn gt boxes into geometry:
    boxes_gt = df_tracking_frame_gt[['bbox_left','bbox_top','bbox_right','bbox_bottom']]
    boxes_gt_geom = []
    for i in range(len(boxes_gt)):
        box_gt = boxes_gt.iloc[i]
        pt1 = [box_gt[0], box_gt[1]]
        pt2 = [box_gt[2], box_gt[1]]
        pt3 = [box_gt[2], box_gt[3]]
        pt4 = [box_gt[0], box_gt[3]] 
        box_gt = Polygon([pt1, pt2, pt3, pt4, pt1])
        boxes_gt_geom.append(box_gt)

    # turn res boxes into geometry:
    boxes_res = df_tracking_frame_res[['bbox_left','bbox_top','bbox_right','bbox_bottom']]
    boxes_res_geom = []
    for i in range(len(boxes_res)):
        box_res = boxes_res.iloc[i]
        pt1 = [box_res[0], box_res[1]]
        pt2 = [box_res[2], box_res[1]]
        pt3 = [box_res[2], box_res[3]]
        pt4 = [box_res[0], box_res[3]] 
        box_res = Polygon([pt1, pt2, pt3, pt4, pt1])
        boxes_res_geom.append(box_res)
    
    selected_boxes_res = []
    selected_boxes_gt = []
    for box_gt in boxes_gt_geom:
        for box_res in boxes_res_geom:
            if box_res.contains(box_gt) or box_res.intersects(box_gt) or box_res.within(box_gt):
                selected_boxes_gt.append(box_gt)
                selected_boxes_res.append(box_res)
    
    # Extract and convert geometry corner points' coordinates to left, top, right, bottom format
    corners_coords_res = []
    for polygon in selected_boxes_res:
        exterior_coords = np.array(polygon.exterior.coords)
        left = np.min(exterior_coords[:, 0])
        right = np.max(exterior_coords[:, 0])
        top = np.min(exterior_coords[:, 1])
        bottom = np.max(exterior_coords[:, 1])
        coords = [left, top, right, bottom]
        corners_coords_res.append(coords)
    corners_coords_res = np.array(corners_coords_res)
    corners_coords_res
        
    return corners_coords_res

def get_boxes_2d(df_tracking_frame_res):
    '''
    function to get boudning box from result without matching ground truth
    '''
    # turn res boxes into geometry:
    boxes_res = df_tracking_frame_res[['bbox_left','bbox_top','bbox_right','bbox_bottom']]
    boxes_res_geom = []
    for i in range(len(boxes_res)):
        box_res = boxes_res.iloc[i]
        pt1 = [box_res[0], box_res[1]]
        pt2 = [box_res[2], box_res[1]]
        pt3 = [box_res[2], box_res[3]]
        pt4 = [box_res[0], box_res[3]] 
        box_res = Polygon([pt1, pt2, pt3, pt4, pt1])
        boxes_res_geom.append(box_res)

    boxes_res = []
    for box_res in boxes_res_geom:
        boxes_res.append(box_res)

    corners_coords_res = []
    for polygon in boxes_res:
        exterior_coords = np.array(polygon.exterior.coords)
        left = np.min(exterior_coords[:, 0])
        right = np.max(exterior_coords[:, 0])
        top = np.min(exterior_coords[:, 1])
        bottom = np.max(exterior_coords[:, 1])
        coords = [left, top, right, bottom]
        corners_coords_res.append(coords)
    corners_coords_res = np.array(corners_coords_res)
    corners_coords_res
        
    return corners_coords_res
