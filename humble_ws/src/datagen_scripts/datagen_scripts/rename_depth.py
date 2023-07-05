import os

path = '/home/danilo/humble_ws/depot_dataset_semantic_nsc_2/depth_data'  # replace with the path to your folder
count = 0
lenght = len(os.listdir(path)) - 201
# Get a list of all file names in the folder
file_names = os.listdir(path)
# Sort the file names numerically based on the number in the name
file_names_sorted = sorted(file_names, key=lambda x: int(x.split('.')[0].split('_')[-1]))
#print(file_names_sorted)
for filename in file_names_sorted:
    #print(filename,new_filename)
    #os.rename(os.path.join(path, filename), os.path.join(path, new_filename))
    if filename == 'dataset_camera::camera_footprint::depth_camera1_'+str(count)+'.png':
        new_filename = f'depth_{lenght}.png'
        lenght += 1
        count += 1	
        #print(filename,new_filename)
        os.rename(os.path.join(path, filename), os.path.join(path, new_filename))	
