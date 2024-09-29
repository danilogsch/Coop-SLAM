#CONSIDERATIONS: 
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from detection_messages.msg import DetectedObjectArray, DetectedObject
from geometry_msgs.msg import PoseArray, Pose
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy

import numpy as np
import os
import cv2
from cv_bridge import CvBridge
import math
import timeit
import yaml

def get_unique_color(unique_id):
    """Generate a visually distinct RGB color based on a unique integer ID."""
    # Calculate color components based on the unique ID and the total number of IDs
    unique_id += 1
    red = (unique_id * 123456) % 256
    green = (unique_id * 987654) % 256
    blue = (unique_id * 543210) % 256
    
    return (red, green, blue)

def twist_and_crop(image, center, yaw_angle, crop_width, crop_height):
    """
    Rotate a grayscale image around a specified point by a yaw angle
    and crop the resulting image with a specified width and height around the same point.

    Args:
    - image: Input grayscale image.
    - center: Tuple (x, y) specifying the center point for rotation and cropping.
    - yaw_angle: Yaw angle in radians for rotation.
    - crop_width: Width of the cropped region.
    - crop_height: Height of the cropped region.

    Returns:
    - Cropped and rotated image.
    """

    # Get image dimensions
    height, width = image.shape[:2]

    # Define rotation matrix for rotation around the specified center
    rotation_matrix = cv2.getRotationMatrix2D(center, np.degrees(yaw_angle), 1)

    # Apply rotation to the image
    rotated_image = cv2.warpAffine(image, rotation_matrix, (width, height), flags=cv2.INTER_NEAREST)

    # Define bounding box for cropping
    x1 = int(center[0] - crop_height / 2)
    x2 = int(center[0] + crop_height / 2)
    y1 = int(center[1] - crop_width / 2)
    y2 = int(center[1] + crop_width / 2)

    # Crop the rotated image
    cropped_image = rotated_image[y1:y2, x1:x2]

    return cropped_image

def twist_and_paste(small_image, big_image, angle, x, y, id, cls, ss_ogm, is_ogm, unique_colors_array, offsets):
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
    if ss_ogm == True:
        big_image_with_rotated_small[indices_with_offset] = unique_colors_array[int(cls+1)]
    elif is_ogm == True:
        big_image_with_rotated_small[indices_with_offset] = unique_colors_array[id]
    else:
        big_image_with_rotated_small[indices_with_offset] = 0
    return big_image_with_rotated_small

def transform_pose(pose, transform):
    x, y, yaw = pose
    tx, ty, t_yaw = transform

    # Apply translation
    new_x = x + np.cos(yaw) * tx - np.sin(yaw) * ty
    new_y = y + np.sin(yaw) * tx + np.cos(yaw) * ty

    # Apply rotation
    new_yaw = yaw - t_yaw
    new_yaw = np.mod(new_yaw + np.pi, 2*np.pi) - np.pi

    return new_x, new_y, new_yaw

def middle_angle(yaw1, yaw2):
    # Bound the yaw values between -π and π
    yaw1 = bound_yaw(yaw1)
    yaw2 = bound_yaw(yaw2)
    
    # Calculate the difference between the two angles
    diff = (yaw2 - yaw1 + math.pi) % (2*math.pi) - math.pi
    
    # Calculate the middle angle
    middle = yaw1 + diff / 2
    
    # Bound the middle angle between -π and π
    middle = bound_yaw(middle)
    
    return middle

def bound_yaw(yaw):
    while yaw > math.pi:
        yaw -= 2 * math.pi
    while yaw < -math.pi:
        yaw += 2 * math.pi
    return yaw


def fix_yaw(t1, t0, yaw1, yaw0, max_vel):
    delta_t = abs(t1-t0)
    delta_yaw = yaw1-yaw0
    delta_yaw = abs(np.mod(delta_yaw+np.pi,2*np.pi)-np.pi)
    recalc_yaw_occ = False
    skip = False
    #if delta_t == 0.0 and delta_yaw < 0.1745:
    #    yaw = yaw1+yaw0/2
    #    yaw = np.mod(yaw+np.pi,2*np.pi)-np.pi
    if delta_yaw > 2.8 and delta_t < 1.1:
        recalc_yaw_occ = True
        yaw = yaw1
    elif delta_yaw > 1.57 and delta_t < 1.1:
        skip = True
        yaw = yaw0
    elif delta_t*max_vel < delta_yaw:
    #elif delta_t < 0.11 and delta_yaw > 0.6:
        yaw = middle_angle(yaw1,yaw0)
        #skip = True
    else:
        yaw = yaw1  
    return yaw, recalc_yaw_occ, skip

class OgmBuilder(Node):

    def __init__(self):
        super().__init__('ogm_builder_node')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('empty_map_path', "/home/danilo/humble_ws/src/coop_detection_and_mapping/map/map_resized.yaml"),
                ('min_conf', 0.3),
                ('symmetric_classes', ['OfficeChairBlue_0','OfficeChairGrey_0','pallet_box_mobile_0','AdjTable_0','aws_robomaker_warehouse_TrashCanC_01_0','cart_model2_0']),
                ('is_ogm', False),
                ('ss_ogm', True),
                ('num_classes', 14),
                ('max_vyaw', 2),
                ('min_pixels_per_occ', 0.05),
                ('x_bound', [-14.0,14.0]),
                ('y_bound', [-7.0,7.0]),
                ('sensor_offset_x', 21.7968),
                ('visualize', True),
                ('publish_map', True),
            ]
        )
        # Get the value of an environment variable
        self.coop_slam_path = os.getenv('COOP_SLAM_PATH')

        if self.coop_slam_path:
            print(f"The path is: {self.coop_slam_path}")
        else:
            print("Environment variable COOP_SLAM_PATH not found.")

        self.empty_map_path = self.get_parameter('empty_map_path').value
        self.min_conf = self.get_parameter('min_conf').value
        self.symmetric_classes = self.get_parameter('symmetric_classes').value
        self.is_ogm = self.get_parameter('is_ogm').value
        self.ss_ogm = self.get_parameter('ss_ogm').value
        self.num_classes = self.get_parameter('num_classes').value
        self.max_vyaw = self.get_parameter('max_vyaw').value
        self.min_pixels_per_occ = self.get_parameter('min_pixels_per_occ').value
        x_bound = self.get_parameter('x_bound').value
        y_bound = self.get_parameter('y_bound').value
        self.min_x = x_bound[0]
        self.max_x = x_bound[1]
        self.min_y = y_bound[0]
        self.max_y = y_bound[1]
        self.sensor_offset_x = self.get_parameter('sensor_offset_x').value
        self.visualize = self.get_parameter('visualize').value
        self.pub_map = self.get_parameter('publish_map').value

        if self.pub_map:
            latching_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.map_publisher = self.create_publisher(OccupancyGrid, 'map', latching_qos)

        if self.ss_ogm == True:
            self.ID_TO_COLOR = [get_unique_color(unique_id) for unique_id in range(self.num_classes+1)]
        elif self.is_ogm == True:
            self.ID_TO_COLOR = [get_unique_color(unique_id) for unique_id in range(50)]
        else:
            self.ID_TO_COLOR = None


        with open(self.empty_map_path, "r") as f:
            f_content = f.read()
        empty_map_params = yaml.safe_load(f_content)
        map_file = empty_map_params['image']
        if map_file[0] != '/':
            map_file = os.path.dirname(self.empty_map_path)+'/'+map_file
        self.empty_map = cv2.imread(map_file, cv2.IMREAD_GRAYSCALE)
        if self.ss_ogm == True or self.is_ogm == True:
            self.empty_map = cv2.cvtColor(self.empty_map, cv2.COLOR_GRAY2RGB)
        
        robot_occ = cv2.imread(self.coop_slam_path+'/humble_ws/instances_footprint/rbkairos_migration_sensor_config_1/footprint_no_arm.png', cv2.IMREAD_GRAYSCALE)
        robot_occ = cv2.resize(robot_occ, None, fx=0.25, fy=0.25, interpolation=cv2.INTER_NEAREST)
        indices = np.where(robot_occ==255)
        self.robot_occ = np.zeros_like(robot_occ, dtype=np.uint8)
        self.robot_occ[indices] = 10

        self.robots = []
        self.last_messages = {}  # Dictionary to store the last received message for each frame_id
        self.tracking_ids = {}

        self.bridge = CvBridge()
        self.cv_window = cv2.namedWindow('Image', cv2.WINDOW_NORMAL)

        self.det_subscription = self.create_subscription(
            PoseArray,
            'detections_topic',
            self.det_callback,
            10)

        self.image_subscription = self.create_subscription(
            Image,
            'seg_out_topic',
            self.image_callback,
            10)
        
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.process_messages)
        self.i = 0
        self.res = 10/608
        self.res2 = empty_map_params['resolution']
        self.map_origin = empty_map_params['origin']
        self.empty_offsets = [round(-empty_map_params['origin'][0]/self.res2), round(self.empty_map.shape[0] + (empty_map_params['origin'][1]/self.res2))]
        print(self.empty_offsets)
        self.time = []


    def det_callback(self, msg):
        #print('received detections')
        frame_id = msg.header.frame_id
        if frame_id not in self.robots:
            self.robots.append(frame_id)
            self.last_messages[frame_id] = {'dets': None, 'image': None}
        if self.last_messages[frame_id]['dets'] == None:
            self.last_messages[frame_id]['dets'] = msg
        #self.process_messages(frame_id)

    def image_callback(self, msg):
        #print('received image')
        frame_id = msg.header.frame_id
        if frame_id not in self.robots:
            self.robots.append(frame_id)
            self.last_messages[frame_id] = {'dets': None, 'image': None}
        if self.last_messages[frame_id]['image'] == None:
            self.last_messages[frame_id]['image'] = msg
        #self.process_messages(frame_id)

    def process_messages(self):
        t0 = timeit.default_timer()
        for frame_id in self.robots:
            
            if (self.last_messages[frame_id]['image'] == None) or (self.last_messages[frame_id]['dets'] == None):
                continue
            cv_image = self.bridge.imgmsg_to_cv2(self.last_messages[frame_id]['image'])
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            #if frame_id == 'rbkairos_migration_sensor_config_3_0':
            #    cv2.imshow('Image', gray_image*10)
            #    cv2.waitKey(1)  # Refresh window
            time = round(self.last_messages[frame_id]['dets'].header.stamp.sec + self.last_messages[frame_id]['dets'].header.stamp.nanosec / 1e9, 4)
            robot_pose = self.last_messages[frame_id]['dets'].poses[0]
            dets = self.last_messages[frame_id]['dets'].poses[1:]
            #print(dets)
            #print(len(dets))
            #print(time)
            #ROBOT POSE -> pose[0]
            if len(self.tracking_ids) == 0:
                self.tracking_ids[0] = {'time': time, 'x': robot_pose.position.x, 'y': robot_pose.position.y, 'ray': 1.5, 'pc':0, 'yaw': robot_pose.orientation.w, 'yaw_pc': 0,'num_pixels': 96, 'cls': 9, 'conf': 1.0, 'occ': self.robot_occ}
            flag = False
            for i in range(len(self.tracking_ids)):
                #print(i)
                #print(self.tracking_ids[i]['x'])
                #print(robot_pose.position.x)
                distance = math.sqrt((robot_pose.position.x - self.tracking_ids[i]['x'])**2 + (robot_pose.position.y - self.tracking_ids[i]['y'])**2)
                if (distance < self.tracking_ids[i]['ray']) and (self.tracking_ids[i]['cls']==9):
                    flag = True
                    self.tracking_ids[i] = {'time': time, 'x': robot_pose.position.x, 'y': robot_pose.position.y, 'ray': 1.5, 'pc':0, 'yaw': robot_pose.orientation.w, 'yaw_pc': 0,'num_pixels': 96, 'cls': 9, 'conf': 1.0, 'occ': self.robot_occ}
            if not flag:
                self.tracking_ids[len(self.tracking_ids)] = {'time': time, 'x': robot_pose.position.x, 'y': robot_pose.position.y, 'ray': 1.5, 'pc':0, 'yaw': robot_pose.orientation.w, 'yaw_pc': 0,'num_pixels': 96, 'cls': 9, 'conf': 1.0, 'occ': self.robot_occ}
            
            #DETECTIONS -> pose[1:]
            for pose in dets:
                #confidence check
                if pose.orientation.x < self.min_conf:
                    #print(f'skipped, confidence lower than minimum')
                    continue
                det_x_m = (pose.position.y + self.sensor_offset_x) * self.res
                det_y_m = (pose.position.x-304) * self.res
                det_ray_m = ((pose.orientation.y+pose.orientation.z)/2) * self.res #(width+length)/2
                det_yaw = pose.orientation.w
                det_cls = pose.position.z

                #if det_cls == 8 and frame_id == 'rbkairos_migration_sensor_config_3_0':
                #    print(det_yaw)

                det_x_m, det_y_m, det_yaw = transform_pose((robot_pose.position.x,robot_pose.position.y, robot_pose.orientation.w),(det_x_m, det_y_m, det_yaw))
                
                det_occ = twist_and_crop(gray_image, (round(pose.position.x/4),round(pose.position.y/4)), pose.orientation.w, math.ceil(pose.orientation.y/4), math.ceil(pose.orientation.z/4))
                indices = np.where(det_occ==det_cls+1)

                if len(indices[0]) < det_occ.shape[0]*det_occ.shape[1]*self.min_pixels_per_occ:
                    continue
                if det_x_m < self.min_x or det_x_m > self.max_x or det_y_m < self.min_y or det_y_m > self.max_y:
                    continue

                #if self.i == 3:
                #    cv2.imwrite('seg_test.png', gray_image.astype(np.uint8)*10+50)
                #    cv2.imwrite('occ_test.png', det_occ.astype(np.uint8)*10+50)
                #
                #self.i += 1

                flag = False
                reid_array = []
                for i in range(len(self.tracking_ids)):
                    #if len(indices[0]) == 0:
                    #    det_occ = self.tracking_ids[i]['occ']
                    #print((abs(time-self.tracking_ids[i]['time']))*self.max_vyaw, abs(det_yaw - self.tracking_ids[i]['yaw']))

                    distance = math.sqrt((det_x_m - self.tracking_ids[i]['x'])**2 + (det_y_m - self.tracking_ids[i]['y'])**2)
                    #print(distance)
                    #if (distance < self.tracking_ids[i]['ray'] or distance < det_ray_m):
                    if (abs(time-self.tracking_ids[i]['time'])) <= 1 and (distance < det_ray_m or distance < self.tracking_ids[i]['ray']):
                        if (det_cls == self.tracking_ids[i]['cls']):
                            if (abs(time-self.tracking_ids[i]['time'])) < 0.11 and 0.5*self.tracking_ids[i]['num_pixels'] > len(indices[0]):
                                flag = True
                                continue
                            if (abs(time-self.tracking_ids[i]['time'])) < 0.1:
                                det_x_m = (det_x_m + self.tracking_ids[i]['x'])/2
                                det_y_m = (det_y_m + self.tracking_ids[i]['y'])/2
                            det_yaw, recalc_yaw_occ, skip = fix_yaw(time, self.tracking_ids[i]['time'], det_yaw, self.tracking_ids[i]['yaw'], self.max_vyaw)
                            if det_cls == 9 or skip == True:
                                flag = True
                                continue
                            if recalc_yaw_occ and self.tracking_ids[i]['yaw_pc'] < 9:
                                #det_yaw = np.mod((pose.orientation.w + np.pi)+np.pi,2*np.pi)-np.pi
                                #det_occ = twist_and_crop(gray_image, (round(pose.position.x/4),round(pose.position.y/4)), det_yaw, math.ceil(pose.orientation.y/4), math.ceil(pose.orientation.z/4))
                                #det_yaw = np.mod((robot_pose.orientation.w -det_yaw)+np.pi,2*np.pi)-np.pi
                                #det_yaw, recalc_yaw_occ = fix_yaw(time, self.tracking_ids[i]['time'], det_yaw, self.tracking_ids[i]['yaw'], self.max_vyaw)
                                det_yaw = np.mod(det_yaw+2*np.pi,2*np.pi)-np.pi #mudar aqui
                                det_occ = self.tracking_ids[i]['occ']
                                yaw_pc = self.tracking_ids[i]['yaw_pc']
                                self.tracking_ids[i] = {'time': time, 'x': det_x_m, 'y': det_y_m, 'ray': det_ray_m, 'pc': 0, 'yaw': det_yaw, 'yaw_pc': 0,'num_pixels': len(indices[0]), 'cls': det_cls, 'conf': pose.orientation.x, 'occ': det_occ}
                                self.tracking_ids[i]['yaw_pc'] = yaw_pc + 1
                                flag = True
                            elif recalc_yaw_occ and self.tracking_ids[i]['yaw_pc'] == 9:
                                #det_yaw = np.mod((robot_pose.orientation.w - pose.orientation.w)+np.pi,2*np.pi)-np.pi
                                self.tracking_ids[i] = {'time': time, 'x': det_x_m, 'y': det_y_m, 'ray': det_ray_m, 'pc': 0, 'yaw': det_yaw, 'yaw_pc': 0,'num_pixels': len(indices[0]), 'cls': det_cls, 'conf': pose.orientation.x, 'occ': det_occ}
                                flag = True
                            else:
                            #print((abs(time-self.tracking_ids[i]['time']))*self.max_vyaw, abs(det_yaw - self.tracking_ids[i]['yaw']))
                                yaw_pc = self.tracking_ids[i]['yaw_pc']
                                self.tracking_ids[i] = {'time': time, 'x': det_x_m, 'y': det_y_m, 'ray': det_ray_m, 'pc': 0, 'yaw': det_yaw, 'yaw_pc': 0,'num_pixels': len(indices[0]), 'cls': det_cls, 'conf': pose.orientation.x, 'occ': det_occ}
                                if yaw_pc > 0:
                                    self.tracking_ids[i]['yaw_pc'] = yaw_pc - 1
                                flag = True
                        elif self.tracking_ids[i]['pc'] == 3: #re-id check
                            #print((abs(time-self.tracking_ids[i]['time']))*self.max_vyaw, abs(det_yaw - self.tracking_ids[i]['yaw']))
                            self.tracking_ids[i] = {'time': time, 'x': det_x_m, 'y': det_y_m, 'ray': det_ray_m, 'pc': 0, 'yaw': det_yaw, 'yaw_pc': 0,'num_pixels': len(indices[0]), 'cls': det_cls, 'conf': pose.orientation.x, 'occ': det_occ}
                            flag = True                            
                        else:
                            self.tracking_ids[i]['pc'] += 1 #re-id check
                            #self.i += 1
                            flag = True
                    if (abs(time-self.tracking_ids[i]['time'])) > 1 and distance < self.tracking_ids[i]['ray'] and (det_cls == self.tracking_ids[i]['cls']):
                        self.tracking_ids[i] = {'time': time, 'x': det_x_m, 'y': det_y_m, 'ray': det_ray_m, 'pc': 0, 'yaw': det_yaw, 'yaw_pc': 0,'num_pixels': len(indices[0]), 'cls': det_cls, 'conf': pose.orientation.x, 'occ': det_occ}
                        flag = True
                        
                if not flag:
                    self.tracking_ids[len(self.tracking_ids)] = {'time': time, 'x': det_x_m, 'y': det_y_m, 'ray': det_ray_m, 'pc': 0, 'yaw': det_yaw, 'yaw_pc': 0,'num_pixels': len(indices[0]), 'cls': det_cls, 'conf': pose.orientation.x, 'occ': det_occ}

            self.last_messages[frame_id] = {'dets': None, 'image': None}

            #print(self.tracking_ids)
            # Create a new dictionary to store unique entries
            unique_dict = {}
            count = 0

            # Iterate over the original dictionary and store unique entries in the new dictionary
            for key, value in self.tracking_ids.items(): #TODO: MOVE to outside
                if value not in unique_dict.values():
                    unique_dict[count] = value
                    print(key)
                    if time - value['time'] > 1: #Increase ray for RE-ID
                        unique_dict[count]['ray'] = unique_dict[count]['ray'] + 0.1
                    count += 1

                # Check if the value is already in the unique dictionary
                #found_duplicate = False
                #for unique_value in unique_dict.values():
                #    if len(unique_dict) == 0:
                #        break
                #    if value['x'] == unique_value['x'] and value['y'] == unique_value['y'] and value['yaw'] == unique_value['yaw'] and value['cls'] == unique_value['cls']:
                #        found_duplicate = True
                #        break
                #    
                #if not found_duplicate:
                #    unique_dict[count] = value
                #    if time - unique_dict[count]['time'] > 1: #Increase ray for RE-ID
                #        unique_dict[count]['ray'] = unique_dict[count]['ray'] + 0.1
                #    count += 1

            self.tracking_ids.clear()  # Clear the original dictionary
            self.tracking_ids.update(unique_dict)  # Update the original dictionary with unique entries
            
            print(unique_dict)
            #print(self.robots)
            #print(self.i)

        ogm = self.empty_map.copy()
        for i in range(len(self.tracking_ids)):
            if self.tracking_ids[i]['occ'] is None or self.tracking_ids[i]['occ'].shape[0] == 0 or self.tracking_ids[i]['occ'].shape[1] == 0:
                continue
            ogm = twist_and_paste(self.tracking_ids[i]['occ'], ogm, self.tracking_ids[i]['yaw'], self.tracking_ids[i]['x'], self.tracking_ids[i]['y'], i, self.tracking_ids[i]['cls'], self.ss_ogm, self.is_ogm, self.ID_TO_COLOR, self.empty_offsets)
        t1 = timeit.default_timer()
        self.time.append(t1-t0)
        if self.visualize:
            cv2.imshow('Image', ogm)
            cv2.waitKey(1)  # Refresh window
        if self.pub_map:
            occ_msg = OccupancyGrid()
            occ_msg.header.stamp = self.get_clock().now().to_msg()
            occ_msg.header.frame_id = 'map'
            occ_msg.info.map_load_time = occ_msg.header.stamp
            occ_msg.info.resolution = self.res2
            occ_msg.info.width = self.empty_map.shape[1]
            occ_msg.info.height = self.empty_map.shape[0]
            #occ_msg.info.origin = self.map_origin
            occ_msg.info.origin.position.x = self.map_origin[0]
            occ_msg.info.origin.position.y = self.map_origin[1]
            #occ_msg.info.origin.orientation.w = 1

            if self.ss_ogm or self.is_ogm:
                ogm = cv2.cvtColor(ogm, cv2.COLOR_RGB2GRAY)
            int8_matrix = np.zeros_like(ogm, dtype=np.int8)

            indices = np.where(ogm != 254)
            int8_matrix[indices] = 100

            indices = np.where(ogm == 205)
            int8_matrix[indices] = -1
            occ_msg.data= np.flip(int8_matrix, axis=0).flatten(order='K').tolist()
            self.map_publisher.publish(occ_msg)


        print(np.mean(self.time))
        print(len(self.tracking_ids))
        #save image and dict
        #if(len(self.tracking_ids)) > 0 and self.i<100:
        #    with open(self.coop_slam_path+f'/humble_ws/src/coop_detection_and_mapping/ogms/poses/{self.i:06d}.txt', 'w') as f:
        #        for i in range(len(self.tracking_ids)):
        #            time = self.tracking_ids[i]['time']
        #            cls = self.tracking_ids[i]['cls']
        #            x = self.tracking_ids[i]['x']
        #            y = self.tracking_ids[i]['y']
        #            yaw = self.tracking_ids[i]['yaw']
        #            f.write(f'{time} {cls} {x} {y} {yaw}\n')
        #    cv2.imwrite(self.coop_slam_path+f'/humble_ws/src/coop_detection_and_mapping/ogms/ogm/{self.i:06d}.png', ogm)
        #    self.i +=1

def main(args=None):
    rclpy.init(args=args)
    ogm_builder_node = OgmBuilder()
    rclpy.spin(ogm_builder_node)
    ogm_builder_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()