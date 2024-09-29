import rclpy
import math
import numpy as np
import cv2
import os
import yaml
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import CostmapFilterInfo
from nav2_msgs.srv import LoadMap
from geometry_msgs.msg import Pose, Point
from tf_transformations import euler_from_quaternion
from ament_index_python.packages import get_package_share_directory


class MapRestricterServer(Node):
    
    def __init__(self):
        super().__init__('map_restricter') 

        self.declare_parameters(
            namespace='',
            parameters=[
                ('cam_x', 0.0),
                ('cam_y', 0.0),
                ('cam_yaw', 0.00)
            ]
        )
        self.i = 0
        self.subscription2 = self.create_subscription(
            Pose,
            'camera_pose',
            self.listener_cp_callback,
            10)
        self.subscription2  # prevent unused variable warning
        self.camera_pose = Pose()
        self.c = 0
        self.map_url_= os.path.join(get_package_share_directory('datagen_scripts'), 'map.yaml')
        self.cli = self.create_client(LoadMap, 'filter_mask_server/load_map')

        while not self.cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().info('service not available, waiting again...X')

        self.req = LoadMap.Request()
        self.response = LoadMap.Response()
        self.apply_restriction()  
        

    def listener_cp_callback(self, msg):

        self.camera_pose = msg
        self.get_logger().info('GOT NEW CAMERA POSE') 
        self.apply_restriction()        

    def apply_restriction(self):
        #Load Map YAML
        with open(self.map_url_, "r") as f:
            file_content = f.read()
        map_meta = yaml.safe_load(file_content)
        map_meta["image"] = os.path.join(get_package_share_directory('datagen_scripts'), map_meta["image"])
        map_img = cv2.imread(map_meta.get("image"), -1)


        # Get Resolution and other info (meters to pixels numbers)
        res = map_meta["resolution"]
        mapwidth = map_img.shape[1]
        mapheight = map_img.shape[0]
        #Camera point
        point_1 = Point()
        point_1.x = float(math.floor((self.get_parameter("cam_x").value - map_meta["origin"][0]) / map_meta["resolution"]))
        point_1.y = float(math.floor((self.get_parameter("cam_y").value - map_meta["origin"][1]) / map_meta["resolution"]))
        point_1.y = map_img.shape[0] - point_1.y

        #q=[self.camera_pose.orientation.x,self.camera_pose.orientation.y,self.camera_pose.orientation.z,self.camera_pose.orientation.w]
        #[r, p, theta] = euler_from_quaternion(q) #rad
        theta = self.get_parameter("cam_yaw").value
        hfov = 120 * math.pi/180 #Camera's horizontal field of view

        angle1 = (float(-theta) + (hfov/2))
        if angle1 > math.pi:
            angle1 = -2*math.pi + angle1

        angle2 = (float(-theta) - (hfov/2))
        if angle2 < -math.pi:
            angle2 = (2*math.pi) + angle2

        camera_dist = 1.5 // res #camera and restriction area distance

        #Points near of camera
        point_2 = Point()
        point_2.x = point_1.x + ((camera_dist* math.cos(angle1)))//1
        point_2.y = point_1.y + ((camera_dist* math.sin(angle1)))//1
        point_3 = Point()
        point_3.x = point_1.x + ((camera_dist* math.cos(angle2)))//1
        point_3.y = point_1.y + ((camera_dist* math.sin(angle2)))//1

        #points_max_dist = (mapwidth + mapheight)//2
        points_max_dist = 10.5 // res

        #Points far of camera
        point_4 = Point()
        point_4.x = point_2.x + ((points_max_dist* math.cos(angle1)))//1
        point_4.y = point_2.y + ((points_max_dist* math.sin(angle1)))//1
        point_5 = Point()
        point_5.x = point_3.x + ((points_max_dist* math.cos(angle2)))//1
        point_5.y = point_3.y + ((points_max_dist* math.sin(angle2)))//1

        # Create mask and draw poly with the calculated points
        mask = np.zeros((mapheight, mapwidth), dtype="uint8")
        polly_points = np.array([ [point_3.x,point_3.y], [point_2.x,point_2.y], [point_4.x,point_4.y], [point_5.x,point_5.y]], np.int)
        cv2.fillConvexPoly(mask, polly_points, 255)

        map_restricted_img = cv2.bitwise_and(map_img, map_img, mask=mask)
        cv2.imwrite(os.path.join(get_package_share_directory('datagen_scripts'), 'map_restricted.pgm'), map_restricted_img)

        #self.get_logger().info('Published filtered masks: "%i"' % self.i)
        self.get_logger().info('Saved filtered map image.')
        self.req.map_url = os.path.join(get_package_share_directory('datagen_scripts'), 'map_restricted.yaml')

        self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    map_restricter = MapRestricterServer()

    rclpy.spin(map_restricter)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    map_restricter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
