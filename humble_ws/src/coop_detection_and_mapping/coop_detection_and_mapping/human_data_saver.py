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



class HumanSaverServer(Node):
    
    def __init__(self):
        super().__init__('human_data_saver') 

        self.bridge = CvBridge()

        self.i = 0
        self.n = 0
        self.g = 0
        
        self.subscription_images = self.create_subscription(
                    Image,
                    'human_vehicle/panoptic/colored_map',
                    self.generic_image_callback,
                    10
                )
        self.subscription_depth = self.create_subscription(
                    Image,
                    'human_vehicle/depth_camera',
                    self.instance_image_callback,
                    10
                )
        self.subscription_images  # prevent unused variable warning
        self.subscription_depth
        # Get the value of an environment variable
        self.coop_slam_path = os.getenv('COOP_SLAM_PATH')

        if self.coop_slam_path:
            print(f"The path is: {self.coop_slam_path}")
        else:
            print("Environment variable COOP_SLAM_PATH not found.")

    def instance_image_callback(self, msg):
        #bridge = CvBridge()
        cv_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        count = len(os.listdir(self.coop_slam_path+'/humble_ws/src/coop_detection_and_mapping/human_data/depth/'))
        file_path = self.coop_slam_path+f'/humble_ws/src/coop_detection_and_mapping/human_data/depth/{count:06d}.exr'  # Specify the file path and name
        cv2.imwrite(file_path,cv_image) 
        #self.g += 1
        print('received_depth')

    def generic_image_callback(self, msg):
        self.g += 1
        print('received_panoptic')

   

def main(args=None):
    rclpy.init(args=args)

    human_data_saver = HumanSaverServer()

    rclpy.spin(human_data_saver)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    human_data_saver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()