import rclpy
from rclpy.duration import Duration
import math
import numpy as np
import cv2
import os
import yaml
import random

from geometry_msgs.msg import PoseStamped
from .robot_navigator import BasicNavigator, TaskResult
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory


class RandomGoalHandler(Node):
    
    def __init__(self):
        super().__init__('random_goal_handler')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('map_yaml_file', os.path.join(get_package_share_directory('datagen_scripts'), 'map_restricted.yaml')),
                ('model_radius', 0.5)
            ]
        )
        self.grid_map=[]
        self.max_x = 0
        self.max_y = 0
        self.resolution = 0
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()
        self.load_map()

    def load_map(self):
        self.grid_map=[]
        with open(self.get_parameter("map_yaml_file").value, "r") as f:
            file_content = f.read()
        map_meta = yaml.safe_load(file_content)
        map_meta["image"] = os.path.join(get_package_share_directory('datagen_scripts'), map_meta["image"])
        map_img = cv2.imread(map_meta.get("image"))
        map_img = np.array(map_img)

        # Anything greater than free_thresh is considered as occupied
        if map_meta["negate"]:
            res = np.where((map_img / 255)[:, :, 0] > map_meta["free_thresh"])
        else:
            res = np.where(((255 - map_img) / 255)[:, :, 0] > map_meta["free_thresh"])

        self.grid_map = np.zeros(shape=(map_img.shape[:2]), dtype=np.uint8)


        for i in range(res[0].shape[0]):
            self.grid_map[res[0][i], res[1][i]] = 255

        self.resolution = map_meta["resolution"]
        
        self.max_x = self.grid_map.shape[1] * map_meta["resolution"] + map_meta["origin"][0]
        self.max_y = self.grid_map.shape[0] * map_meta["resolution"] + map_meta["origin"][1]

        self.calculate_and_send_waypoints()

    def calculate_and_send_waypoints(self):
        goal_poses = []

        for i in range(2):
            is_valid_pose = False
            while (is_valid_pose == False):

                #RANDOM INITIAL POSES (->string)
                x = round((random.random()*self.max_x)-(random.random()*self.max_x), 4)
                y = round((random.random()*self.max_y)-(random.random()*self.max_y), 4)
                #z = round(random.random(), 4)
                yaw = round((random.random()*-1.570796327) + (random.random()*1.570796327), 4)

                ##Check if its a valid pose in the Grid map
                i_x = math.floor((x + self.max_x) / self.resolution)
                i_y = math.floor((y + self.max_y) / self.resolution)
                i_y = self.grid_map.shape[0] - i_y

                distance = math.ceil((self.get_parameter("model_radius").value + 0.55)/ self.resolution)

                row_start_idx = 0 if i_y - distance < 0 else i_y - distance
                col_start_idx = 0 if i_x - distance < 0 else i_x - distance

                patch = self.grid_map[row_start_idx : i_y + distance, col_start_idx : i_x + distance]
                obstacles = np.where(patch == 255)
                if len(obstacles[0]) == 0:
                    
                    is_valid_pose = True
                    # Draw circle with model radius
                    model_circle = np.zeros(shape=(self.grid_map.shape[:2]), dtype=np.uint8)
                    cv2.circle(model_circle, (i_x, i_y), distance, 255, -1)
                    # Update grid map
                    self.grid_map = cv2.bitwise_or(self.grid_map,model_circle)

            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x = float(x)
            goal_pose.pose.position.y = float(y)
            [goal_pose.pose.orientation.x,goal_pose.pose.orientation.y,goal_pose.pose.orientation.z,goal_pose.pose.orientation.w] = self.get_quaternion_from_euler(0,0,float(yaw))
            goal_poses.append(goal_pose)

        nav_start = self.navigator.get_clock().now()
        self.navigator.followWaypoints(goal_poses)

        i = 0
        while not self.navigator.isTaskComplete():
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Executing current waypoint: ' +
                      str(feedback.current_waypoint + 1) + '/' + str(len(goal_poses)))
                now = self.navigator.get_clock().now()

                # Some navigation timeout to demo cancellation
               # if now - nav_start > Duration(seconds=10.0):
                    #self.navigator.cancelTask()

        # Do something depending on the return code
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')
        
        self.load_map()

#        navigator.lifecycleShutdown()
#
#        exit(0)


    def get_quaternion_from_euler(self, roll, pitch, yaw):
        """
          Convert an Euler angle to a quaternion.
        
          Input
            :param roll: The roll (rotation around x-axis) angle in radians.
            :param pitch: The pitch (rotation around y-axis) angle in radians.
            :param yaw: The yaw (rotation around z-axis) angle in radians.
        
          Output
            :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
        """
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)

    random_goal_handler = RandomGoalHandler()

    #rclpy.spin(random_goal_handler)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    #random_goal_handler.destroy_node()
    #rclpy.shutdown()


if __name__ == '__main__':
    main()
