#!/usr/bin/env python3

import threading
import random
import math

import rclpy
from .moveit2 import MoveIt2Interface
from geometry_msgs.msg import Quaternion
import time
from math import sin, cos, pi


#refer https://index.ros.org/doc/ros2/Tutorials/URDF/Using-URDF-with-Robot-State-Publisher/
def euler_to_quaternion(roll, pitch, yaw):
  qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
  qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
  qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
  qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
  return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main(args=None):
    rclpy.init(args=args)

    # Initialise MoveIt2
    moveit2 = MoveIt2Interface()

    # Spin MoveIt2 node in the background
    executor = rclpy.executors.MultiThreadedExecutor(1)
    executor.add_node(moveit2)
    thread = threading.Thread(target=executor.spin)
    thread.start()

    # Set pose goal to reach
    while(1): 
        #position = [-0.000001, 0.500001, 0.600001]
        #position = round(position, 6)
        #quaternion = euler_to_quaternion(0, 0, 0)
        #quaternion = Quaternion(x=0.0, y= 0.0, z= 0.000001,w=0.999999)
        #quaternion = round(quaternion,6)
        #moveit2.set_pose_goal(position, quaternion)
        j1_p = round(((random.random() * math.pi/2) - (random.random() * math.pi/2)),2)
        j2_p = round((-(random.random() * math.pi/4) - math.pi/4),2)
        j3_p = round(( - (random.random() * math.pi)),2)
        j4_p = round(((random.random() * math.pi/2) - (random.random() * math.pi/2)),2)
        j5_p = round(((random.random() * math.pi/2) - (random.random() * math.pi/2)),2)
        j6_p = round(((random.random() * math.pi/2) - (random.random() * math.pi/2)),2)
        joint_positions = [j1_p,j2_p,j3_p,j4_p,j5_p,j6_p]
        #joint_positions = round(joint_positions, 6)
        moveit2.set_joint_goal(joint_positions)
        
        # Plan and execute
        moveit2.plan_kinematic_path()
        moveit2.execute()
        time.sleep(3)
        #joint_positions = [0.0,0.0,0.0,0.0,0.0,0.0]
        ##joint_positions = round(joint_positions, 6)
        #moveit2.set_joint_goal(joint_positions)
        ## Plan and execute
        #moveit2.plan_kinematic_path()
        #moveit2.execute()
        #time.sleep(5)
   
    rclpy.shutdown()


if __name__ == "__main__":
    main()
