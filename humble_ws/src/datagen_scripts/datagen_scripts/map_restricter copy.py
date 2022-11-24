import rclpy
import math
import numpy as np
import cv2
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import CostmapFilterInfo
from nav_msgs.srv import GetMap
from geometry_msgs.msg import Pose, Point
from tf_transformations import euler_from_quaternion


class MapRestricterServer(Node):
    
    def __init__(self):
        super().__init__('map_restricter')
        self.map_publisher_ = self.create_publisher(OccupancyGrid, 'filter_mask', 5)
        self.map_publisher_info_ = self.create_publisher(CostmapFilterInfo, 'costmap_filter_info', 5)
        self.rest_map_msg = OccupancyGrid()
        self.filter_info_msg = CostmapFilterInfo()
        self.filter_info_msg.base = np.float(0)
        self.filter_info_msg.multiplier = np.float(1)
        self.filter_info_msg.type = 0
        self.filter_info_msg.filter_mask_topic = 'filter_mask'
        #timer_period = 0.5  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback())
        self.i = 0
        self.subscription2 = self.create_subscription(
            Pose,
            'camera_pose',
            self.listener_cp_callback,
            10)
        self.subscription2  # prevent unused variable warning
        self.camera_pose = Pose()
        self.c = 0
        self.cli = self.create_client(GetMap, '/map_server/map')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.i =1 #self.get_logger().info('service not available, waiting again...')
        self.get_logger().info('service available')
        self.req = GetMap.Request()
        self.response = GetMap.Response()
        self.send_request()
        

    def listener_cp_callback(self, msg):
        
        if (self.c == 0):
            self.camera_pose = msg
            self.get_logger().info('GOT CAMERA POSE') 
            self.c = 1
            self.apply_restriction()        

    def apply_restriction(self):      
        # Get Resolution and other info (meters to pixels numbers)
        res = self.response.map.info.resolution
        mapwidth = self.response.map.info.width
        mapheight = self.response.map.info.height
        #Camera point
        point_1 = Point()
        point_1.x = (-self.response.map.info.origin.position.x // res) + (self.camera_pose.position.x // res) #pixel
        point_1.y = (-self.response.map.info.origin.position.y // res) + (self.camera_pose.position.y // res) #pixel

        q=[self.camera_pose.orientation.x,self.camera_pose.orientation.y,self.camera_pose.orientation.z,self.camera_pose.orientation.w]
        [r, p, theta] = euler_from_quaternion(q) #rad
        hfov = 90 * math.pi/180 #Camera's horizontal field of view

        angle1 = (theta + (hfov/2))
        if angle1 > math.pi:
            angle1 = -2*math.pi + angle1

        angle2 = (theta - (hfov/2))
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

        points_max_dist = (mapwidth + mapheight)//2

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

        self.rest_map_msg.data = self.response.map.data #Resize
        for y in range(mapheight):
            for x in range(mapwidth):
                if (mask[y,x] == 255):
                    map_cell=np.int8(0)
                elif (mask[y,x] == 0):
                    map_cell=np.int8(100)
                elif (mask[y,x]!=0) and (mask[x,y]!=255):
                    map_cell=np.int8(-1)
                #Fill Occupancy grid data
                self.rest_map_msg.data[mapwidth * (mapheight - y - 1) + x] = map_cell

        self.rest_map_msg.header.frame_id = self.response.map.header.frame_id
        self.rest_map_msg.info = self.response.map.info
        self.rest_map_msg.header.stamp = self.get_clock().now().to_msg()
        #self.rest_map_msg.header.frame_id = 'map'
        self.filter_info_msg.header.frame_id = self.rest_map_msg.header.frame_id
        self.filter_info_msg.header.stamp = self.get_clock().now().to_msg()
        self.map_publisher_.publish(self.rest_map_msg)
        self.map_publisher_info_.publish(self.filter_info_msg)
        self.get_logger().info('Published filtered masks: "%i"' % self.i)





    #def timer_callback(self):
    #    #Update Headers & info
    #    
    #    self.rest_map_msg.header.frame_id = self.response.map.header.frame_id
    #    self.rest_map_msg.info = self.response.map.info
    #    self.rest_map_msg.header.stamp = self.get_clock().now().to_msg()
    #    #self.rest_map_msg.header.frame_id = 'map'
    #    self.filter_info_msg.header.frame_id = self.rest_map_msg.header.frame_id
    #    self.filter_info_msg.header.stamp = self.get_clock().now().to_msg()
    #    #self.map_publisher_.publish(self.rest_map_msg)
    #    #self.map_publisher_info_.publish(self.filter_info_msg)
    #    self.get_logger().info('Published filtered masks: "%i"' % self.i)
    #    self.i += 1

    def send_request(self):
        self.future = self.cli.call_async(self.req)
        while rclpy.ok():
            rclpy.spin_once(self)
            if self.future.done():
                self.response = self.future.result()
                self.get_logger().info('GOT MAP')
                break

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
