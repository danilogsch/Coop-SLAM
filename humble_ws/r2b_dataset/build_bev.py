import os
import cv2
import numpy as np
import matplotlib.pyplot as plt
import time
import pandas as pd
import math
import argparse

boundary = {
    "minX": 0,
    "maxX": 10,
    "minY": -5,
    "maxY": 5,
    "minZ": -0.6,
    "maxZ": 2.4
}

bound_size_x = boundary['maxX'] - boundary['minX']
bound_size_y = boundary['maxY'] - boundary['minY']
bound_size_z = boundary['maxZ'] - boundary['minZ']

BEV_WIDTH = 608  # across y axis -5m ~ 5m
BEV_HEIGHT = 608  # across x axis 0m ~ 10m
DISCRETIZATION = bound_size_x / BEV_HEIGHT 

def get_filtered_lidar(lidar, boundary, labels=None):
    minX = boundary['minX']
    maxX = boundary['maxX']
    minY = boundary['minY']
    maxY = boundary['maxY']
    minZ = boundary['minZ'] 
    maxZ = boundary['maxZ']
    #minZ = np.min(lidar[:,2])
    #print(minZ)
    #print(np.max(lidar[:,2]))
    #maxZ = minZ + 3

    # Remove the point out of range x,y,z
    mask = np.where((lidar[:, 0] >= minX) & (lidar[:, 0] <= maxX) &
                    (lidar[:, 1] >= minY) & (lidar[:, 1] <= maxY) &
                    (lidar[:, 2] >= minZ) & (lidar[:, 2] <= maxZ))
    lidar = lidar[mask]
    lidar[:, 2] = lidar[:, 2] - minZ

    if labels is not None:
        label_x = (labels[:, 1] >= minX) & (labels[:, 1] < maxX)
        label_y = (labels[:, 2] >= minY) & (labels[:, 2] < maxY)
        label_z = (labels[:, 3] >= minZ) & (labels[:, 3] < maxZ)
        mask_label = label_x & label_y & label_z
        labels = labels[mask_label]
        return lidar, labels
    else:
        return lidar
    
def makeBEVMap(PointCloud_, boundary):
    Height = BEV_HEIGHT + 1
    Width = BEV_WIDTH + 1

    # Discretize Feature Map
    PointCloud = np.copy(PointCloud_)
    PointCloud[:, 0] = np.int_(np.floor(PointCloud[:, 0] / DISCRETIZATION))
    PointCloud[:, 1] = np.int_(np.floor(PointCloud[:, 1] / DISCRETIZATION) + Width / 2)

    # sort-3times
    sorted_indices = np.lexsort((-PointCloud[:, 2], PointCloud[:, 1], PointCloud[:, 0]))
    PointCloud = PointCloud[sorted_indices]
    _, unique_indices, unique_counts = np.unique(PointCloud[:, 0:2], axis=0, return_index=True, return_counts=True)
    PointCloud_top = PointCloud[unique_indices]

    # Height Map, Intensity Map & Density Map
    heightMap = np.zeros((Height, Width))
    intensityMap = np.zeros((Height, Width))
    densityMap = np.zeros((Height, Width))

    # some important problem is image coordinate is (y,x), not (x,y)
    max_height = float(np.abs(boundary['maxZ'] - boundary['minZ'])) #3.00
    heightMap[np.int_(PointCloud_top[:, 0]), np.int_(PointCloud_top[:, 1])] = PointCloud_top[:, 2] / max_height

    normalizedCounts = np.minimum(1.0, np.log(unique_counts + 1) / np.log(64))
    #intensityMap[np.int_(PointCloud_top[:, 0]), np.int_(PointCloud_top[:, 1])] = PointCloud_top[:, 3]
    densityMap[np.int_(PointCloud_top[:, 0]), np.int_(PointCloud_top[:, 1])] = normalizedCounts

    BGR_Map = np.zeros((3, Height - 1, Width - 1))
    BGR_Map[2, :, :] = densityMap[:BEV_HEIGHT, :BEV_WIDTH]  # r_map
    BGR_Map[1, :, :] = heightMap[:BEV_HEIGHT, :BEV_WIDTH]  # g_map
    BGR_Map[0, :, :] = intensityMap[:BEV_HEIGHT, :BEV_WIDTH]  # b_map

    return BGR_Map

#print (lidar_file)
#t0 = time.time()
i = 0
lidar_file_list = os.listdir('/home/danilo/r2b_dataset/r2b_hallway_lidar')
lidar_files = sorted([file for file in lidar_file_list if file.lower().endswith('.bin')])
#base_run_path = '/home/danilo/r2b_dataset'

for lidar_file in lidar_files[:]:

    lidar_file_path = os.path.join('/home/danilo/r2b_dataset/r2b_hallway_lidar', lidar_file)
    data = np.fromfile(lidar_file_path, dtype=np.float32).reshape(-1, 3)
#t1 = time.time()
    data = get_filtered_lidar(data, boundary)
#t2 = time.time()
    bev_map = makeBEVMap(data, boundary) #to tensor
#t3 = time.time()
    bev_array = (np.transpose(bev_map,(1,2,0))* 255).astype(np.uint8) #to display
#t4 = time.time()
#print(f'Loading time: {t1-t0}. Filter time: {t2 -t1}. BEV build time: {t3-t2}. Transpose array time: {t4-t3}')
#lidar.append(data)
#print(data)
    cv2.imwrite(f'/home/danilo/r2b_dataset/r2b_hallway_bev/{i:06d}.png', bev_array)
    print(f'Saved file: bev_maps/{i:06d}.png')
    i += 1
