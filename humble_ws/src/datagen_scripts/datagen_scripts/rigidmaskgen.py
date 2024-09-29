import os
os.environ["OPENCV_IO_ENABLE_OPENEXR"]="1"
import cv2
import numpy as np
import matplotlib.pyplot as plt

# Specify the folder path containing PNG images
base_model_path = '/home/danilo/humble_ws/data_dump/0/instances/'
model_names = [
'PatientWalkingCane',
'MaleVisitorPhone',
'FemaleVisitorWalk',
'MaleVisitor',
'OpScrubsWalk',
'VisitorKidWalk',
'NurseFemaleWalk',
'DoctorFemaleWalk',
'TrashBin',
'OfficeChairBlue',
'OfficeChairGrey',
'pallet_box_mobile',
'AdjTable',
'aws_robomaker_warehouse_TrashCanC_01',
'cart_model2',
'aws_robomaker_warehouse_PalletJackB_01',
'youbot',
'rbkairos_migration_sensor_config_1',
'pioneer3dx_migration_sensor_config_1',
'robotino',
'rbkairos_migration_sensor_config_2',
'pioneer3at',
'Mecanum_lift']

for model_name in model_names:
# List all files in the folder
#file_list = os.listdir(base_model_path+'/'+model_name+'/segmentation_data/instance_camera/labels_maps')

#depth_file_list = os.listdir(base_model_path+'/'+model_name+'/depth_data')
#depth_file_list = os.listdir('/home/danilo/humble_ws/data_dump')

# Filter the list to include only PNG files
#png_files = [file for file in file_list if file.lower().endswith('.png')]
#depth_png_files = [file for file in depth_file_list if file.lower().endswith('.exr')]
# Load each PNG image as a NumPy array and store them in a list
#loaded_images = []
#loaded_depths = []
    max_depth = 10
    min_depth = 0.1

    label_img = cv2.imread(f'{base_model_path}{model_name}/segmentation_data/instance_camera/labels_maps/labels_0000005.png')
    depth_img = cv2.imread(f'{base_model_path}{model_name}/depth/000005.exr', cv2.IMREAD_UNCHANGED)

    height, width = label_img.shape[:2]
    middle_x = width // 2
    middle_y = height // 2
    max_depth = 10
    min_depth = 0.1

    # Get the pixel value at the middle pixel
    middle_pixel_value = label_img[middle_y, middle_x]
    blanck_pixel_value = label_img[0,0]

    print(f"Pixel value at the middle pixel: {middle_pixel_value}. blank: {blanck_pixel_value}")

    # Define the target pixel value [1, 0, 1]
    target_value = np.array([1, 0, 1])

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

    cropped_mask = mask[min_y+75:max_y+75,min_x+75:max_x+75]
    cv2.imwrite(f'{base_model_path}{model_name}/footprint.png',cropped_mask)

    # Display the mask
    plt.imshow(cropped_mask, cmap='gray', vmin=0, vmax=255)
    plt.title('Mask')
    plt.colorbar()
    plt.show()

