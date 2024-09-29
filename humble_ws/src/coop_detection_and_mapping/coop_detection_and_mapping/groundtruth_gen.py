import rclpy
import math
import os
os.environ["OPENCV_IO_ENABLE_OPENEXR"]="1"
import pandas as pd
from rclpy.node import Node
import numpy as np

from tf_transformations import euler_from_quaternion
from ament_index_python.packages import get_package_share_directory
import cv2
import yaml

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

def get_unique_color(unique_id):
    """Generate a visually distinct RGB color based on a unique integer ID."""
    # Calculate color components based on the unique ID and the total number of IDs
    unique_id += 1
    red = (unique_id * 123456) % 256
    green = (unique_id * 987654) % 256
    blue = (unique_id * 543210) % 256
    
    return (red, green, blue)

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

def twist_and_paste(small_image, big_image, angle, x, y, cls, unique_colors_array, offsets):
    # Get dimensions of smaller image
    #0 value represents background in segmentation image
    small_height, small_width = small_image.shape[:2]

    # Compute the size of the canvas needed to contain the rotated image
    max_dim = max(small_height, small_width)
    canvas_size = (max_dim * 2, max_dim * 2)

    # Create a blank canvas
    canvas = np.zeros(canvas_size, dtype=np.uint8)

    # Compute the position to paste the smaller image onto the canvas
    offset_x = (canvas_size[1] - small_width) // 2
    offset_y = (canvas_size[0] - small_height) // 2

    # Paste the smaller image onto the canvas
    canvas[offset_y:offset_y + small_height, offset_x:offset_x + small_width] = small_image

    # Compute center of the canvas
    center = (canvas.shape[1] // 2, canvas.shape[0] // 2)

    # Compute rotation matrix
    rotation_matrix = cv2.getRotationMatrix2D(center, np.degrees(angle+(math.pi/2)), 1.0)

    # Rotate canvas
    rotated_canvas = cv2.warpAffine(canvas, rotation_matrix, (canvas.shape[1], canvas.shape[0]), flags=cv2.INTER_NEAREST)

    # Compute offset to paste rotated canvas onto larger image
    offset_x = offsets[0] + round(x/0.065789474) - (rotated_canvas.shape[1] // 2)
    offset_y = offsets[1] - round(y/0.065789474) - (rotated_canvas.shape[0] // 2)

    #u, v = np.where(rotated_canvas == (id))+(offset_y,offset_x)
    indices = np.where(rotated_canvas == (cls+1))

    indices_with_offset = (indices[0] + offset_y, indices[1] + offset_x)
  
    # Paste rotated canvas onto larger image
    big_image_with_rotated_small = big_image.copy()

    big_image_with_rotated_small[indices_with_offset] = unique_colors_array[int(cls+1)]

    return big_image_with_rotated_small

class GroundTruthGenerator(Node):
    
    def __init__(self):
        super().__init__('groundtruth_gen') 
        self.declare_parameters(
            namespace='',
            parameters=[
                ('empty_map_path', "/home/danilo/humble_ws/src/coop_detection_and_mapping/map/map_resized.yaml"),
                ('save_path', "/home/danilo/humble_ws/src/coop_detection_and_mapping/groundtruths"),
                ('used_models', ['rbkairos_migration_sensor_config_3_0','rbkairos_migration_sensor_config_3_1','rbkairos_migration_sensor_config_3_2','human_vehicle', 'youbot', 'pallet_box_mobile', 'aws_robomaker_warehouse_PalletJackB_01', 'cart_model2' ])#['rbkairos_migration_sensor_config_3_0','rbkairos_migration_sensor_config_3_1','rbkairos_migration_sensor_config_3_2','TrashBin','OfficeChairBlue','AdjTable','aws_robomaker_warehouse_TrashCanC_01','pioneer3dx_migration_sensor_config_1','pioneer3dx_migration_sensor_config_2','pioneer3at','robotino','Mecanum_lift'])
            ]
        )
        # Get the value of an environment variable
        self.coop_slam_path = os.getenv('COOP_SLAM_PATH')

        if self.coop_slam_path:
            print(f"The path is: {self.coop_slam_path}")
        else:
            print("Environment variable COOP_SLAM_PATH not found.")
        self.empty_map_path = self.get_parameter('empty_map_path').value
        self.models_used = self.get_parameter('used_models').value
        self.save_pt = self.get_parameter('save_path').value
        with open(self.empty_map_path, "r") as f:
            f_content = f.read()
        empty_map_params = yaml.safe_load(f_content)
        map_file = empty_map_params['image']
        if map_file[0] != '/':
            map_file = os.path.dirname(self.empty_map_path)+'/'+map_file
        self.empty_map = cv2.imread(map_file, cv2.IMREAD_GRAYSCALE)
        self.empty_map = cv2.cvtColor(self.empty_map, cv2.COLOR_GRAY2RGB)
        self.cv_window = cv2.namedWindow('Image', cv2.WINDOW_NORMAL)

        self.ID_TO_COLOR = [get_unique_color(unique_id) for unique_id in range(15)]
        self.CLASS_NAME_TO_ID = {
            'human_vehicle': 0,
            'TrashBin': 1,
            'OfficeChairBlue': 2,
            'pallet_box_mobile': 3,
            'AdjTable': 4,
            'aws_robomaker_warehouse_TrashCanC_01': 5,
            'cart_model2': 6,
            'aws_robomaker_warehouse_PalletJackB_01': 7,
            'youbot': 8,
            'rbkairos_migration_sensor_config_3_0': 9,
            'rbkairos_migration_sensor_config_3_1': 9,
            'rbkairos_migration_sensor_config_3_2': 9,
            'pioneer3dx_migration_sensor_config_1': 10,
            'pioneer3dx_migration_sensor_config_2': 10,
            'robotino': 11,
            'pioneer3at': 12,
            'Mecanum_lift': 13
        }

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        #self.bridge = CvBridge()

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.transform_listen)

        self.i = 0
        self.n = 0
        self.g = 0  
        print('initialized')    

    def transform_listen(self):
        if self.n <101:
            df = pd.DataFrame(columns=['time', 'cls', 'x', 'y', 'yaw'])
            occ_map = self.empty_map.copy()
            for name in self.models_used:
                try:
                    t = self.tf_buffer.lookup_transform(
                            'empty',
                            name,
                            rclpy.time.Time())
                except TransformException as ex:
                    self.get_logger().info(
                            f'Could not transform {name} to empty: {ex}')
                    return
                _, _, yaw = quaternion_to_euler(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w)
                new_row = {
                 'time': round(t.header.stamp.sec + t.header.stamp.nanosec / 1e9, 4),
                 'cls': self.CLASS_NAME_TO_ID[name],
                 'x': t.transform.translation.x,
                 'y': t.transform.translation.y,
                 'yaw': yaw,
                }
                df.loc[len(df.index)] = new_row
                if name == 'human_vehicle': ##check aux_data_saver and bev_map_gen to understand human segmentation label
                    label_img = cv2.imread(self.coop_slam_path+f'/humble_ws/src/coop_detection_and_mapping/human_data/panoptic/labels_maps/labels_{self.n:07d}.png')
                    depth_img = cv2.imread(self.coop_slam_path+f'/humble_ws/src/coop_detection_and_mapping/human_data/depth/{self.n:06d}.exr', cv2.IMREAD_UNCHANGED)
                    target_value = np.array(label_img[300,400])
                    mask = np.all(label_img == target_value, axis=-1)
                    indices = np.argwhere(mask)
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
                    model_occ = mask
                    model_occ = cv2.resize(model_occ, None, fx=0.25, fy=0.25, interpolation=cv2.INTER_NEAREST)
                    #continue
                elif name[:6] == 'rbkair':
                    occ_path = self.coop_slam_path+'/humble_ws/instances_footprint/rbkairos_migration_sensor_config_1/footprint_no_arm.png'
                    model_occ = cv2.imread(occ_path, cv2.IMREAD_GRAYSCALE)
                    model_occ = cv2.resize(model_occ, None, fx=0.25, fy=0.25, interpolation=cv2.INTER_NEAREST)
                elif name == 'pioneer3dx_migration_sensor_config_2' and self.n >=59:
                    occ_path = self.coop_slam_path+'/humble_ws/instances_footprint/pioneer3dx_migration_sensor_config_1/footprint.png'
                    model_occ = cv2.imread(occ_path, cv2.IMREAD_GRAYSCALE)
                    model_occ = cv2.resize(model_occ, None, fx=0.25, fy=0.25, interpolation=cv2.INTER_NEAREST)
                elif name == 'pioneer3dx_migration_sensor_config_1' and self.n < 28:
                    occ_path = self.coop_slam_path+'/humble_ws/instances_footprint/pioneer3dx_migration_sensor_config_1/footprint.png'
                    model_occ = cv2.imread(occ_path, cv2.IMREAD_GRAYSCALE)
                    model_occ = cv2.resize(model_occ, None, fx=0.25, fy=0.25, interpolation=cv2.INTER_NEAREST)
                elif name == 'pioneer3dx_migration_sensor_config_1' or name == 'pioneer3dx_migration_sensor_config_2':
                    continue
                else:
                    occ_path = self.coop_slam_path+f'/humble_ws/instances_footprint/{name}/footprint.png' 
                    model_occ = cv2.imread(occ_path, cv2.IMREAD_GRAYSCALE)
                    model_occ = cv2.resize(model_occ, None, fx=0.25, fy=0.25, interpolation=cv2.INTER_NEAREST)
                indices = np.where(model_occ==255)
                occ = np.zeros_like(model_occ, dtype=np.uint8)
                occ[indices] = self.CLASS_NAME_TO_ID[name]+1
                occ_map = twist_and_paste(occ, occ_map, yaw, t.transform.translation.x, t.transform.translation.y, self.CLASS_NAME_TO_ID[name], self.ID_TO_COLOR, [231, 115])
                #df = df.append(new_row, ignore_index=True)
            # Save the DataFrame to a .txt file with space-separated values
            file_path = self.coop_slam_path+f'/humble_ws/src/coop_detection_and_mapping/groundtruths/poses/{self.n:06d}.txt'  # Specify the file path and name
            df.to_csv(file_path, sep=' ', index=False)
            file_path = self.coop_slam_path+f'/humble_ws/src/coop_detection_and_mapping/groundtruths/ogm/{self.n:06d}.png'
            cv2.imwrite(file_path, occ_map)
            cv2.imshow('Image', occ_map)
            cv2.waitKey(1)  # Refresh window
            self.n += 1


def main(args=None):
    rclpy.init(args=args)

    groundtruth_gen = GroundTruthGenerator()

    rclpy.spin(groundtruth_gen)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    groundtruth_gen.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()