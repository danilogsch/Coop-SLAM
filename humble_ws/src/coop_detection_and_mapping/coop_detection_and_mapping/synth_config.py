import math

import numpy as np


CLASS_NAME_TO_ID = {
    'person': 0,
    'trash_bin': 1,
    'chair': 2,
    'pallet_box': 3,
    'table': 4,
    'trash_can': 5,
    'cart': 6,
    'pallet_jack': 7,
    'youbot': 8,
    'rbkairos': 9,
    'p3dx': 10,
    'robotino': 11,
    'p3at': 12,
    'lift': 13
}

colors = [[0, 215, 255], [180,130,70], [237,149,100], [255,191,0],
          [255,144,30], [230,216,173], [235,206,135], [255,105,65],
          [170,205,102], [113,179,60], [170,178,32], [139,139,0],
          [87,139,43], [50,205,50]]

#####################################################################################
boundary = {
    "minX": 0,
    "maxX": 10,
    "minY": -5,
    "maxY": 5,
    "minZ": -0.5, #calcular relativo a cada .bin
    "maxZ": 3
}

bound_size_x = boundary['maxX'] - boundary['minX']
bound_size_y = boundary['maxY'] - boundary['minY']
bound_size_z = boundary['maxZ'] - boundary['minZ']

boundary_back = {
    "minX": -10,
    "maxX": 0,
    "minY": -5,
    "maxY": 5,
    "minZ": 0,
    "maxZ": 3
}

BEV_WIDTH = 608  # across y axis -25m ~ 25m
BEV_HEIGHT = 608  # across x axis 0m ~ 50m
DISCRETIZATION = (boundary["maxX"] - boundary["minX"]) / BEV_HEIGHT

# maximum number of points per voxel
T = 35

# voxel size
vd = 0.1  # z
vh = 0.05  # y
vw = 0.05  # x

# voxel grid
W = math.ceil(bound_size_x / vw)
H = math.ceil(bound_size_y / vh)
D = math.ceil(bound_size_z / vd)

Tr_velo_to_cam = np.array([
    [0, -1, 0, 0],
    [0, 0, -1, 0],
    [1, 0, 0, 0],
    [0, 0, 0, 1]
])


R0 = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
]) 


P2 = np.array([[537.61, 0.0, 960.0, 0.0],
               [0.0, 537.61, 600.0, 0.0],
               [0.0, 0.0, 1.0, 0.0]
               ])

R0_inv = np.linalg.inv(R0)
Tr_velo_to_cam_inv = np.linalg.inv(Tr_velo_to_cam)
P2_inv = np.linalg.pinv(P2)
#####################################################################################