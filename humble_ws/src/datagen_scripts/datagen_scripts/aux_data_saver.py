import rclpy
import math
import os
os.environ["OPENCV_IO_ENABLE_OPENEXR"]="1"
import pandas as pd
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from sensor_msgs_py import point_cloud2
import numpy as np
from geometry_msgs.msg import Pose, Point
from sensor_msgs.msg import PointCloud2
from tf_transformations import euler_from_quaternion
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
import cv2


from functools import partial

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener



class AuxSaverServer(Node):
    
    def __init__(self):
        super().__init__('aux_data_saver') 
        self.declare_parameters(
            namespace='',
            parameters=[
                ('run_number', 68),
                ('save_path', "data_dump"),
                ('used_models', ['OfficeChairBlue_0','OfficeChairGrey_0','pallet_box_mobile_0','AdjTable_0','aws_robomaker_warehouse_TrashCanC_01_0','cart_model2_0']),
                ('save_lidar', True),
                ('save_depth', False)
            ]
        )

        self.models_used = self.get_parameter('used_models').value
        self.run_n = self.get_parameter('run_number').value
        self.save_pt = self.get_parameter('save_path').value
        self.sv_lidar = self.get_parameter('save_lidar').value
        self.sv_depth = self.get_parameter('save_depth').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.bridge = CvBridge()

        self.i = 0
        self.n = 0
        self.g = 0      
        
        # Subscribers
        if self.sv_depth == True :
            os.makedirs(str(self.save_pt)+'/'+str(self.run_n)+'/depth_data/', exist_ok=True)
            self.subscription_depth = self.create_subscription(
            Image,
            '/depth_camera',
            self.depth_callback,
            20
            )
            self.subscription_depth  # prevent unused variable warning

        if self.sv_lidar==True:
            os.makedirs(f'{self.save_pt}/{self.run_n}/instances/poses/', exist_ok=True)
            os.makedirs(str(self.save_pt)+'/'+str(self.run_n)+'/lidar/', exist_ok=True)
            self.subscription_lidar = self.create_subscription(
            PointCloud2,
            '/lidar/points',
            self.pointcloud_callback,
            10
            )
            self.subscription_lidar  # prevent unused variable warning
        
        if len(self.models_used) > 0:

            for i in range(len(self.models_used)):
                if self.models_used[i][-4:] == '_cam':
                    self.models_used[i] = self.models_used[i][:-4]
                    os.makedirs(f'{self.save_pt}/{self.run_n}/instances/{self.models_used[i]}/depth/', exist_ok=True)
                    self.subscription_images = self.create_subscription(
                                Image,
                                str(self.models_used[i])+'/panoptic/colored_map',
                                self.generic_image_callback,
                                10
                            )
                    self.subscription_depth = self.create_subscription(
                                Image,
                                str(self.models_used[i])+'/depth_camera',
                                partial(self.instance_image_callback, index = i),
                                10
                            )
                    #self.subscription_images  # prevent unused variable warning
                    self.subscription_depth


    def depth_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        cv2.imwrite(str(self.save_pt)+'/'+str(self.run_n)+f'/depth_data/{self.i:06d}.exr',cv_image)
        

        
    def pointcloud_callback(self, msg):
        # Extract point cloud data
        pc_data = point_cloud2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True)

        # Convert point cloud data to a NumPy array
        pc_array = np.array(list(pc_data), dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32), ('intensity', np.float32)])

        # Flatten the 2D array into a 1D array
        pc_flat = pc_array.flatten()

        # Save the 1D array to a binary .bin file
        pc_flat.tofile(str(self.save_pt)+'/'+str(self.run_n)+'/lidar/'+f"{self.i:06d}"+'.bin')
        self.i += 1
        self.transform_listen(self.models_used)

    def transform_listen(self, models_used):
        df = pd.DataFrame(columns=['name', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
        for name in models_used:
            try:
                t = self.tf_buffer.lookup_transform(
                        'dataset_camera',
                        name,
                        rclpy.time.Time())
            except TransformException as ex:
                self.get_logger().info(
                        f'Could not transform {name} to empty: {ex}')
                return
            new_row = {
             'name': name,
             'x': t.transform.translation.x,
             'y': t.transform.translation.y,
             'z': t.transform.translation.z,
             'qx': t.transform.rotation.x,
             'qy': t.transform.rotation.y,
             'qz': t.transform.rotation.z,
             'qw': t.transform.rotation.w,
            }
            df.loc[len(df.index)] = new_row
            #df = df.append(new_row, ignore_index=True)
        # Save the DataFrame to a .txt file with space-separated values
        file_path = f'{self.save_pt}/{self.run_n}/instances/poses/{self.n:06d}.txt'  # Specify the file path and name
        df.to_csv(file_path, sep=' ', index=False)
        self.n += 1


    def instance_image_callback(self, msg, index):
        #bridge = CvBridge()
        cv_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        count = len(os.listdir(f'{self.save_pt}/{self.run_n}/instances/{self.models_used[index]}/depth/'))
        file_path = f'{self.save_pt}/{self.run_n}/instances/{self.models_used[index]}/depth/{count:06d}.exr'  # Specify the file path and name
        cv2.imwrite(file_path,cv_image) 
        #cv2.imwrite('data_dump/teste.tif',cv_image) 
        #print("recebi imagem")
        #self.g += 1
        print('received_depth')

    def generic_image_callback(self, msg):
        self.g += 1
        print('received_panoptic')

   

def main(args=None):
    rclpy.init(args=args)

    aux_data_saver = AuxSaverServer()

    rclpy.spin(aux_data_saver)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    aux_data_saver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()