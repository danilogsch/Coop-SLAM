import os
os.environ["OPENCV_IO_ENABLE_OPENEXR"]="1"
import cv2
import numpy as np
import matplotlib.pyplot as plt
import time
import pandas as pd
import math
import argparse

parser = argparse.ArgumentParser(description='My Script Description')
parser.add_argument('--run', type=int, default=0, help='Run number')
parser.add_argument('--base_path', type=str, default='/home/danilo/humble_ws/data_dump/', help='Base Path')
parser.add_argument('--save_bev', action='store_true', help='Save .png BEV')
parser.add_argument('--save_seg', action='store_true', help='Save .png sem. segmentation labels')
parser.add_argument('--vis', action='store_true', help='Vizualize annotation iteractively')
args = parser.parse_args()

# Get the value of an environment variable
coop_slam_path = os.getenv('COOP_SLAM_PATH')

if coop_slam_path:
    print(f"The path is: {coop_slam_path}")
else:
    print("Environment variable COOP_SLAM_PATH not found.")

def quaternion_to_euler(x, y, z, w):
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

# Example usage:
#x, y, z, w = 0.0, 0.0, 0.0, 1.0  # Replace with your quaternion values
#roll, pitch, yaw = quaternion_to_euler(x, y, z, w)
#print(f"Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}")

# bev image coordinates format
def get_corners(x, y, w, l, yaw):
    trackletBox = np.array([  # in bev coordinates around zero point and without orientation yet
            [w / 2, w / 2, -w / 2, -w / 2], \
            [-l / 2, l / 2, l / 2, -l / 2], \
            [0, 0, 0, 0]])
    rotMat = np.array([
            [np.cos(yaw), -np.sin(yaw), 0.0],
            [np.sin(yaw), np.cos(yaw), 0.0],
            [0.0, 0.0, 1.0]])
    res = 10/608
    translation = [x/res, y/res+304, 0]
    
    cornerPosInVelo = np.dot(rotMat, trackletBox) + np.tile(translation, (4, 1)).T
    box3d = cornerPosInVelo.transpose()

    return box3d

def drawRotatedBox(img, x, y, w, l, yaw, color):
    # Convert the points to a NumPy array
    points = np.delete(get_corners(x,y,w,l,yaw).astype(np.int32), 2, axis=1)[:, ::-1]
    cv2.polylines(img, [points], True, color, 1)
    #corners_int = bev_corners.reshape(-1, 2)
    cv2.line(img, (int(points[0, 0]), int(points[0, 1])), (int(points[1, 0]), int(points[1, 1])), (255, 255, 0), 1)


def get_filtered_lidar(lidar, boundary, labels=None):
    minX = boundary['minX']
    maxX = boundary['maxX']
    minY = boundary['minY']
    maxY = boundary['maxY']
    minZ = np.min(lidar[:,2])
    maxZ = minZ + 3

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
    intensityMap[np.int_(PointCloud_top[:, 0]), np.int_(PointCloud_top[:, 1])] = PointCloud_top[:, 3]
    densityMap[np.int_(PointCloud_top[:, 0]), np.int_(PointCloud_top[:, 1])] = normalizedCounts

    BGR_Map = np.zeros((3, Height - 1, Width - 1))
    BGR_Map[2, :, :] = densityMap[:BEV_HEIGHT, :BEV_WIDTH]  # r_map
    BGR_Map[1, :, :] = heightMap[:BEV_HEIGHT, :BEV_WIDTH]  # g_map
    BGR_Map[0, :, :] = intensityMap[:BEV_HEIGHT, :BEV_WIDTH]  # b_map

    return BGR_Map 

# Specify the folder path containing PNG images
run = args.run
print(f'STARTED RUN {run}')
base_run_path = args.base_path+str(run)+'/'

#Read DataFrame file:
df = pd.read_csv(coop_slam_path+'/humble_ws/src/datagen_scripts/gz_configs/data_frame_dict.csv', index_col='Name')

lidar_file_list = os.listdir(base_run_path+'lidar')
lidar_files = sorted([file for file in lidar_file_list if file.lower().endswith('.bin')])
lidar = []
labels_list = []
os.makedirs(f'{base_run_path}bev_maps', exist_ok=True)
os.makedirs(f'{base_run_path}annotations', exist_ok=True)
os.makedirs(f'{base_run_path}seg_labels', exist_ok=True)

camera_pose_txt = base_run_path+'camera_pose.txt'
with open(camera_pose_txt, "r") as file:
    # Read all lines into a list
    camera_yaw = float(file.readline().strip().split()[2])
    #print(f'CAMERA YAW {camera_yaw}')


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
#print(lidar_files)
i = 0
for lidar_file in lidar_files[:]:
    pose_txt_file = base_run_path+f'instances/poses/{i:06d}.txt'
    
    df_3dbbox = pd.read_csv(base_run_path+f'bounding_box_3d_data/boxes/boxes_{i:07d}.csv')
    inst_list = df_3dbbox['label'].tolist()
    labels = []
    o = 0
    actors_count = 1
    rbkairos_count = 1
    youbot_count = 1
    line_idx=0
    black_list=[]
    seg_img = np.zeros((608, 608), dtype=np.uint8)

    # Open the file in read mode
    with open(pose_txt_file, "r") as file:
    # Read all lines into a list
        lines = file.readlines()

    for line in lines[1:]:
            #print(line)
            line = line.rstrip()
            line_parts = line.split(' ')
            if line_parts[0][:14] == 'human_vehicle_':
                 line_parts[0] = line_parts[0][14:]
            obj_name = line_parts[0][:-2]
            #print(obj_name)
            #print(df["label_id"][obj_name])
            #serch label_id on dict
            if df["class_id"][obj_name] not in inst_list:
                #print('OCLUSION DETECTED')
                continue
            cat_id = int(df["label_id"][obj_name])-1
            #translation from pose 
            x, y, z = float(line_parts[1]), float(line_parts[2]), float(line_parts[3]) #(continue if out of boundary)
            l, w = cv2.imread(coop_slam_path+'/humble_ws/instances_footprint'+f'/{obj_name}/footprint.png').shape[:2] #base_footprint (y-axis,x-axis)
            shadow = cv2.imread(coop_slam_path+'/humble_ws/instances_footprint'+f'/{obj_name}/150.png', cv2.IMREAD_GRAYSCALE)
            h = float(df['height'][obj_name]) #model_height (rigid.py)
            if cat_id == 0 or cat_id == 8 or cat_id == 9:
                #yaw = pitch from bbox3d +90 degrees (check angle limits) (row o) (skip if object not found)
                #check number of lines
                
                flag = False
                negate = False
                #line_idx=0
                #black_list=[]

                while flag == False:
                    #print(f'O= {o}, idx={line_idx} len 3d bb= {len(df_3dbbox.index)}')
                    if line_idx == len(df_3dbbox.index):
                        continue

                    row = df_3dbbox.iloc[line_idx]
                    row_list = row.tolist()
                    #print(f'row list= {int(row_list[0])}, df class = {int(df["class_id"][obj_name])}')
                    if int(row_list[0]) == int(df["class_id"][obj_name]):
                        if line_idx not in black_list:
                            black_list.append(line_idx)
                            flag = True
                    line_idx += 1

                #row = df_3dbbox.iloc[o]

                # Convert the row to a Python list (array)
                #row_list = row.tolist()
                if int(row_list[0]) != 20 and int(row_list[0]) != 10 and int(row_list[0]) != 11:
                    continue
                if cat_id == 0:
                    #print(float(row_list[8]))
                    if float(row_list[7]) > 0:
                        rz = 1.5708 - float(row_list[8])
                    elif float(row_list[7]) < 0:
                        rz = -1.5708 + float(row_list[8])
                    #print(f'passei aqui {rz} {camera_yaw}')
                    if rz > math.pi:
                        rz = -2 * math.pi + rz
                        #print(f'passei aqui {rz}')
                else:
                    _, _, rz = quaternion_to_euler(float(line_parts[4]), float(line_parts[5]), float(line_parts[6]), float(line_parts[7]))

                # l, w, h from top depth cam
                if cat_id == 0:
                    label_img = cv2.imread(base_run_path+f'instances/human_vehicle_{line_parts[0]}/instance_data/labels_maps/labels_{i:07d}.png')
                    depth_img = cv2.imread(base_run_path+f'instances/human_vehicle_{line_parts[0]}/depth/{i:06d}.exr', cv2.IMREAD_UNCHANGED)
                else:
                    label_img = cv2.imread(base_run_path+f'instances/{line_parts[0]}/instance_data/labels_maps/labels_{i:07d}.png')
                    depth_img = cv2.imread(base_run_path+f'instances/{line_parts[0]}/depth/{i:06d}.exr', cv2.IMREAD_UNCHANGED)
                
                target_value = np.array(label_img[300,400])
                ## Define the target pixel value [cls_id, 0 , inst_count]
                #if cat_id == 0:
                #    target_value = np.array([20, 0, int(actors_count)])
                #    actors_count += 1
                #elif cat_id == 8:
                #    target_value = np.array([10, 0, int(youbot_count)])
                #    youbot_count += 1
                #elif cat_id == 9:
                #    target_value = np.array([11, 0, int(rbkairos_count)])
                #    rbkairos_count += 1

                # Create a boolean mask to identify matching pixels
                mask = np.all(label_img == target_value, axis=-1)

                # Use the boolean mask to get the indices of matching pixels
                indices = np.argwhere(mask)

                # Print the indices
                #print(indices)
                fx = 400.31867027282715
                fy = 400.31867027282715
                cx = 400.0
                cy = 300.0
                points = []
                points_x = []
                points_y = []
                depths = []
                mask = np.zeros((150,150), dtype="uint8")
                for u, v in indices:
                    depth = depth_img[u,v]
                    depths.append(depth)
                    point_x = (v - cx) * depth / fx
                    point_x_pixel = int(point_x // 0.016447368)
                    point_y = (u - cy) * depth / fy
                    point_y_pixel = int(point_y // 0.016447368)
                    mask[75-point_y_pixel,75-point_x_pixel] = 255
                    points.append([point_x, point_y])
                    points_x.append(-point_x_pixel)
                    points_y.append(-point_y_pixel)
                    #print(f"Meters: {point_x_pixel}, {point_y_pixel}")
                    #print(len(points))
                
                #if len(points_x) == 0 or len(points_y) == 0:
                #    o += 1
                #    continue
                max_x = max(points_x)
                max_y = max(points_y)
                min_x = min(points_x)
                min_y = min(points_y)

                if -min_x > max_x: 
                    max_x = -min_x 
                else:
                    min_x = -max_x
                if -min_y > max_y:
                    max_y = -min_y
                else:
                    min_y = -max_y 
                #print(f'MAX X {max_x}, Y {max_y}, MIN X {min_x}, Y {min_y}')
                #print(f'MAX X {max_x+75}, Y {max_y+75}, MIN X {min_x+75}, Y {min_y+75}')

                shadow = mask

                cropped_mask = mask[min_y+75:max_y+75,min_x+75:max_x+75]
                l, w = cropped_mask.shape[:2]
                h = depth_img.max() - min(depths)

            else:
                _, _, rz = quaternion_to_euler(float(line_parts[4]), float(line_parts[5]), float(line_parts[6]), float(line_parts[7]))

            _, _, seg_angle = quaternion_to_euler(float(line_parts[4]), float(line_parts[5]), float(line_parts[6]), float(line_parts[7]))
            center = (shadow.shape[1] // 2, shadow.shape[0] // 2)
            rotation_matrix = cv2.getRotationMatrix2D(center, (seg_angle*180/math.pi), 1.0)
            translation_matrix = np.array([[0, 0, (y//(10/608))+304-75], [0, 0, (x//(10/608))-75]])
            combined_matrix = rotation_matrix + translation_matrix
            rotated_shadow = cv2.warpAffine(shadow, combined_matrix, (608, 608))
            for j in range(rotated_shadow.shape[1]):
                for k in range(rotated_shadow.shape[0]):
                    if rotated_shadow[j,k] == 255:
                        seg_img[j,k] = cat_id+1
        
            object_label = [cat_id, x, y, z, l, w, h, rz]
            #print(object_label)
            labels.append(object_label)
            o += 1
    sv_bev = args.save_bev
    if sv_bev == True:
        #print (lidar_file)
        t0 = time.time()
        lidar_file_path = os.path.join(base_run_path, 'lidar', lidar_file)
        data = np.fromfile(lidar_file_path, dtype=np.float32).reshape(-1, 4)
        t1 = time.time()
        data = get_filtered_lidar(data, boundary)
        t2 = time.time()
        bev_map = makeBEVMap(data, boundary) #to tensor
        t3 = time.time()
        bev_array = (np.transpose(bev_map,(1,2,0))* 255).astype(np.uint8) #to display
        t4 = time.time()
        print(f'Loading time: {t1-t0}. Filter time: {t2 -t1}. BEV build time: {t3-t2}. Transpose array time: {t4-t3}')
        lidar.append(data)
        #print(data)
        cv2.imwrite(f'{base_run_path}bev_maps/{i:06d}.png', bev_array)
        print(f'Saved file: bev_maps/{i:06d}.png')

    sv_seg = args.save_seg
    if sv_seg == True:
        cv2.imwrite(f'{base_run_path}seg_labels/{i:06d}.png', seg_img)
        print(f'Saved file: seg_labels/{i:06d}.png')



    with open(f'{base_run_path}annotations/{i:06d}.txt', "w") as file:
        # Iterate through the list of lists
        for sublist in labels:
            # Convert each sublist to a string and write it to the file
            file.write(" ".join(map(str, sublist)) + "\n")
    print(f'Saved file: annotations/{i:06d}.txt')
    
    if args.vis == True and os.path.isfile(f'{base_run_path}bev_maps/{i:06d}.png')==0:
        print('BEV images do not exist in path. Use --save_bev true')
        exit
    elif args.vis == True:
        bev_array_read = cv2.imread(f'{base_run_path}bev_maps/{i:06d}.png')
        for object in labels:
        #print(object)
        #print(float(object[1]))
            drawRotatedBox(bev_array_read, float(object[1]), float(object[2]), float(object[4]), float(object[5]), float(object[7]), color=([255, 255, 255]))
    i += 1
    #print(type(bev_array_read))
    
    
    if args.vis == True:
        imshow_obj = plt.imshow(bev_array_read, vmin=0, vmax=255)
        plt.title('BEV-OBB (Press any key to iterate)')
        #plt.colorbar()
        plt.axis('off')  # Hide axis labels and ticks (optional)
        plt.show(block=False)
        #Wait for a key press or close the window to continue
        while not plt.waitforbuttonpress():
            pass
        #Close the Matplotlib window
        #plt.close()
        imshow_obj.remove()

