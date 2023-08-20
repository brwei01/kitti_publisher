#!/usr/bin/python3

import cv2
import numpy as np
import pandas as pd

TRACKING_COL_NAMES = ['frame', 'left', 'top', 'right', 'bottom', 'distance']

def read_camera(path):
    return cv2.imread(path)

def read_point_cloud(path):
    return np.fromfile(path, dtype=np.float32).reshape(-1,4)

def read_tracking(path):
    df = pd.read_csv(path, header=None, sep=' ')
    df.columns = TRACKING_COL_NAMES
    return df
    
