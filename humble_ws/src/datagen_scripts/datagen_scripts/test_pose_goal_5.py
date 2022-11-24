#!/usr/bin/env python3

import threading
import random
import math

import rclpy
from .moveit2_5 import MoveIt2Interface
import time

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
        j1_p = round((random.random() * 5.9),2)
        j2_p = round((random.random() * 2.70),2)
        j3_p = round(( - (random.random() * math.pi)),2)
        j4_p = round((random.random() * 3.58 ),2)
        j5_p = round((random.random() * 5.85 ),2)
        joint_positions = [j1_p,j2_p,j3_p,j4_p,j5_p]

        moveit2.set_joint_goal(joint_positions)
        
        # Plan and execute
        moveit2.plan_kinematic_path()
        moveit2.execute()
        time.sleep(3)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
