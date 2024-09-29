import os
import shutil

# Function to move files according to the described pattern
def move_images(source_dir, destination_dir):
    if not os.path.exists(destination_dir):
        os.makedirs(destination_dir)
    index = 0
    # Iterate over the source directory
    for i in range(0, 101):
        # Move the first image to the destination folder
        
        src_path = os.path.join(source_dir, f"{index:06d}.png")
        dst_path = os.path.join(destination_dir, f"{index:06d}.png")
        shutil.move(src_path, dst_path)
        index = index+3


# Replace 'source_folder' and 'destination_folder' with your actual paths
source_folder = "r2b_hallway_rgb"
destination_folder = "r2b_hallway_rgb/sync_imgs"

move_images(source_folder,destination_folder)
