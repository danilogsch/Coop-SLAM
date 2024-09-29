import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from detection_messages.msg import DetectedObjectArray, DetectedObject
from geometry_msgs.msg import PoseArray, Pose
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from sensor_msgs_py import point_cloud2 as pc2

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import numpy as np
import math
import timeit

import argparse
import sys
import os
import time
import warnings
import zipfile

warnings.filterwarnings("ignore", category=UserWarning)

import cv2
from cv_bridge import CvBridge
import torch

from .model_utils import create_model
from .evaluation_utils import draw_predictions, convert_det_to_real_values
from .synth_config import bound_size_y, bound_size_x, BEV_WIDTH, BEV_HEIGHT, colors, boundary, DISCRETIZATION
from .demo_utils import parse_demo_configs, do_detect
from .synth_bev_utils import makeBEVMap
from .synth_data_utils import get_filtered_lidar

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

class DetectionNode(Node):

    def __init__(self):
        super().__init__('detection_node')
        self.configs = parse_demo_configs()
        # Get the value of an environment variable
        self.coop_slam_path = os.getenv('COOP_SLAM_PATH')

        if self.coop_slam_path:
            print(f"The path is: {self.coop_slam_path}")
        else:
            print("Environment variable COOP_SLAM_PATH not found.")

        self.configs.pretrained_path = self.coop_slam_path+'/humble_ws/checkpoints/fpn_resnet_50_semseg/Model_fpn_resnet_50_semseg_epoch_262.pth'
        self.model = create_model(self.configs)
        print('\n\n' + '-*=' * 30 + '\n\n')
        assert os.path.isfile(self.configs.pretrained_path), "No file at {}".format(self.configs.pretrained_path)
        self.model.load_state_dict(torch.load(self.configs.pretrained_path, map_location='cuda:0'))
        print('Loaded weights from {}\n'.format(self.configs.pretrained_path))
        self.configs.device = torch.device('cpu' if self.configs.no_cuda else 'cuda:{}'.format(self.configs.gpu_idx))
        self.model = self.model.to(device=self.configs.device)
        self.model.eval()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.seg_publisher = self.create_publisher(Image, 'seg_out_topic', 10)
        #self.det_publisher = self.create_publisher(DetectedObjectArray, 'detections_topic', 10)
        self.det_publisher = self.create_publisher(PoseArray, 'detections_topic', 10)

        self.subscription = self.create_subscription(
            PointCloud2,
            '/rbkairos_migration_sensor_config_3_1/lidar/points',
            self.listener_callback,
            1)
        self.subscription  # prevent unused variable warning
        self.bridge_ = CvBridge()
        #self.clock = Clock()
        #self.publisher_ = self.create_publisher(String, 'topic', 10)
        #timer_period = 0.5  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.t1_array = []
        self.t2_array = []
        self.t3_array = []
        self.t4_array = []

    

    def listener_callback(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg.data)
        t0 = timeit.default_timer()
        #self.get_logger().info('Got points')
        gen = []
        for p in pc2.read_points(msg, field_names = ("x", "y", "z"), skip_nans=True):
                gen.append(np.array([p[0], p[1], p[2], 0]))#p[3]/100.0]))
        gen_numpy = np.array(gen, dtype=np.float32)
        t1 = timeit.default_timer()
        front_lidar = get_filtered_lidar(gen_numpy, boundary)
        t2 = timeit.default_timer()
        bev_map = makeBEVMap(front_lidar, boundary)
        bev_map = torch.from_numpy(bev_map)
        t3 = timeit.default_timer()
        
        with torch.no_grad():
            detections, bevmap, fps, seg_out = do_detect(self.configs, self.model, bev_map)
        #print(fps)
        
        #print(f'Array time: {t1-t0}. Filter time: {t2 -t1}. BEV build time: {t3-t2}.')
        #print('Prediction time: ', (1/fps))
        #print('Total time: ', (t3 - t0) +(1/fps))
        if self.i > 0:
            self.t1_array.append(t1-t0)
            self.t2_array.append(t2-t1)
            self.t3_array.append(t3-t2)
            self.t4_array.append((1/fps))
        if self.i == 2:
            print(detections)
        self.i+= 1
        #objects_msg = DetectedObjectArray()
        objects_msg = PoseArray()
        objects_msg.header.stamp = self.get_clock().now().to_msg()
        robot_name = msg.header.frame_id.split('/')
        objects_msg.header.frame_id = str(robot_name[0])

        obj = Pose()

        try:
            t = self.tf_buffer.lookup_transform(
                        'empty',
                        str(robot_name[0]),
                        rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                        f'Could not transform {robot_name[0]} to empty: {ex}')
            return

        #objects_msg.posx = t.transform.translation.x
        #objects_msg.posy = t.transform.translation.y
        #objects_msg.orientation.x = t.transform.rotation.x
        #objects_msg.orientation.y = t.transform.rotation.y
        #objects_msg.orientation.z = t.transform.rotation.z
        #objects_msg.orientation.w = t.transform.rotation.w

        _, _, yaw = quaternion_to_euler(t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w)

        obj.orientation.w = float(yaw)
        obj.position.x = float(t.transform.translation.x)
        obj.position.y = float(t.transform.translation.y)

        objects_msg.poses.append(obj)

        #print(f'{t.transform.translation.y}')

        flag = False
        for j in range(self.configs.num_classes):
            if len(detections[j]) > 0:
                flag = True
                for det in detections[j]:
                    _score, _x, _y, _z, _h, _w, _l, _yaw = det
                    #obj = DetectedObject()
                    obj = Pose()
                    obj.position.x = float(_x)
                    obj.position.y = float(_y)
                    obj.position.z = float(j)
                    obj.orientation.x = float(_score)
                    obj.orientation.y = float(_w)
                    obj.orientation.z = float(_l)
                    obj.orientation.w = float(_yaw)
                    #obj.category = j
                    #obj.confidence = float(_score)
                    #obj.x = float(_x)
                    #obj.y = float(_y)
                    #obj.width = float(_w)
                    #obj.lenght = float(_l)
                    #obj.yaw = float(_yaw)


                    objects_msg.poses.append(obj)

        # Create a ROS Image message
        #image_message = Image()
        seg_out = cv2.cvtColor(seg_out.astype(np.uint8), cv2.COLOR_GRAY2RGB)
        image_message = self.bridge_.cv2_to_imgmsg(seg_out, 'bgr8')

        image_message.header.stamp = self.get_clock().now().to_msg()
        image_message.header.frame_id = str(robot_name[0])

        #print(image_message.header.stamp)
        #print(msg.header.stamp)
        #print(rclpy.time.Time())

        # Publish the image message
        if flag is True:
            self.seg_publisher.publish(image_message)
            self.det_publisher.publish(objects_msg)
            print('TIMES: ', np.mean(self.t1_array), np.mean(self.t2_array), np.mean(self.t3_array), np.mean(self.t4_array) )



def main(args=None):
    rclpy.init(args=args)
    detection_node = DetectionNode()

    rclpy.spin(detection_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    detection_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()