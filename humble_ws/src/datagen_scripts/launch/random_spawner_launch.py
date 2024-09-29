#from ast import Not
#from distutils.spawn import spawn
import os
import random
import pandas
from sympy import true
import yaml
#from PIL import Image
import numpy as np
import math
import cv2
from collections import Counter

from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import EmitEvent, SetEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription, LogInfo, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.conditions import IfCondition
from launch.events import Shutdown
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_common.launch import RewrittenYaml


def get_quaternion_from_euler(roll, pitch, yaw):
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

def count_folders_in_directory(directory_path):
    # Initialize a counter to keep track of the number of folders
    folder_count = 0
    # Check if the directory exists, and create it if it doesn't
    if not os.path.exists(directory_path):
        os.makedirs(directory_path)

    # Check if the directory exists
    if os.path.exists(directory_path) and os.path.isdir(directory_path):
        # List all items in the directory
        items = os.listdir(directory_path)

        # Iterate through the items and count the folders
        for item in items:
            item_path = os.path.join(directory_path, item)
            if os.path.isdir(item_path):
                folder_count += 1
    else:
        print(f"The directory '{directory_path}' does not exist.")

    return folder_count

def generate_launch_description():

    launch_params_path = os.path.join(get_package_share_directory('datagen_scripts'), 'launch_params.yaml')

    with open(launch_params_path, "r") as f:
        f_content = f.read()
    launch_params = yaml.safe_load(f_content)

    map_server_dcl = DeclareLaunchArgument(
            'map',
            default_value=os.path.join(get_package_share_directory('datagen_scripts'), 'map.yaml'),
            description='Full path to map yaml file to load')

    os.environ['GZ_LAUNCH_CONFIG_PATH'] = launch_params['launch_config_path']
    launch_path = launch_params['launch_config_path']

    os.environ['IGN_GAZEBO_RESOURCE_PATH'] = launch_params['model_resouce_path']
    resource_path = launch_params['model_resouce_path']

    log_settings = LaunchConfiguration("log_settings", default="true")

    map_restricted_filepath = os.path.join(get_package_share_directory('datagen_scripts'), 'map_restricted.yaml')
    map_filepath = os.path.join(get_package_share_directory('datagen_scripts'), 'map.yaml')

    cam_x = float(launch_params['camera']['x'])
    cam_y = float(launch_params['camera']['y'])
    cam_yaw = float(launch_params['camera']['yaw'])

    use_cam_config_pose = launch_params['camera']['use_cam_config_pose']
    if use_cam_config_pose == False:
        valid_cam_pose = False
        while valid_cam_pose == False:
            cam_x = round(random.random()*16.00 - 8.00,4)
            cam_y = round(random.random()*12.00 - 6.00,4)

            if cam_x >= 0 and cam_x <= 8 and cam_y >= -2 and cam_y <= 2:
                cam_yaw = random.random() * math.pi/9 + 2.9670
                valid_cam_pose = True

            if cam_x >= -8 and cam_x < 0 and cam_y >= -2 and cam_y <= 2:
                cam_yaw = random.random() * math.pi/9 - 0.1745
                valid_cam_pose = True
                
            if cam_x >= -6 and cam_x <= 6 and cam_y >= -6 and cam_y <= -3:
                cam_yaw = random.random() * math.pi/9 + 1.3962
                valid_cam_pose = True

            if cam_x >= -6 and cam_x <= 6 and cam_y >= 3 and cam_y <= 6:
                cam_yaw = random.random() * math.pi/9 + 4.5378
                valid_cam_pose = True

            if cam_yaw > math.pi:
                cam_yaw = -2*math.pi + cam_yaw

            cam_yaw = round(cam_yaw,4)

    print('CAMERA', cam_x, cam_y, cam_yaw)

    cam_pose_list = [cam_x, cam_y, cam_yaw]
    
    [qx, qy, qz, qw] = get_quaternion_from_euler(0, 0, cam_yaw)
    print(r'"{position: {x: '+str(cam_x)+',y: '+str(cam_y)+'}, orientation: {x: '+str(qx)+', y: '+str(qy)+', z: '+str(qz)+', w: '+str(qw)+r'}}" -1')

    models = [] #List of Robots
    models_a = [] #List of Actors
    models_o = [] #List of Objects

    used_models_r = [] #List of used models in this run
    used_models_a = []
    used_models_o = []

    run_number = count_folders_in_directory(str(launch_params['camera']['save_path']))

    os.makedirs(str(launch_params['camera']['save_path'])+f'/{run_number}', exist_ok=True)

    with open(str(launch_params['camera']['save_path'])+f'/{run_number}/camera_pose.txt', 'w') as file:
        file.write(' '.join(map(str, cam_pose_list)))

    #if run_number!=0 :
    #    run_number+=1

    n_max_r = launch_params['n_max_r']
    n_max_a = launch_params['n_max_a']
    n_max_o = launch_params['n_max_o']

    #random.seed(632651)

    #Read DataFrame file:
    df = pandas.read_csv(launch_path+'/data_frame_dict.csv', index_col='Name')

    n_max_i = n_max_r+n_max_a+n_max_o

    df = df.sort_values(
        by=['type','n_uses', 'label_id']
        )
    print(df)
    n_classes = max(df['label_id'][name] for name in df.index)

    # Create a DataFrame with custom index and a "cls_counter" column
    data = {"cls_counter": [0] * n_classes}

    cls_counter_df = pandas.DataFrame(data, index=range(1, n_classes+1))

    for cls in range(1, n_classes+1):
        uses_counter = sum(df['n_uses'][name] for name in df.index if df['label_id'][name]==cls)
        cls_counter_df.loc[cls,'cls_counter'] = uses_counter
    
    cls_counter_df = cls_counter_df.sort_values(
        by=['cls_counter']
        )

    #Fill String lists
    for name in df.index:
        if df['type'][name] == 'r':
            models.append(name)
        elif df['type'][name] == 'a':
            models_a.append(name)
        elif df['type'][name] == 'o':
            models_o.append(name)


    cls_counter_df = cls_counter_df[:n_max_i]

    for cls in cls_counter_df.index:
        
        for name in df.index:
            if (df['label_id'][name] == cls):
                
                if df['type'][name]=='r' and len(used_models_r)<n_max_r:
                    used_models_r.append(name)
                    df.loc[name,'n_uses'] += 1
                    continue
                
                if df['type'][name]=='a' and len(used_models_a)<n_max_a:
                    used_models_a.append(name)
                    df.loc[name,'n_uses'] += 1
                    continue
                
                if df['type'][name]=='o' and len(used_models_o)<n_max_o:
                    used_models_o.append(name)
                    df.loc[name,'n_uses'] += 1
                    continue


    used_models_r_n = used_models_r
    count_r = Counter(used_models_r)
    counter = 0
    for i in range(len(used_models_r)):
        print(used_models_r[i])
        if count_r[used_models_r[i]] >1:
            used_models_r_n[i] = used_models_r[i]+'_'+str(counter)
            counter += 1
        elif count_r[used_models_r[i]] ==1:
            counter = 0
            used_models_r_n[i] = used_models_r[i]+'_'+str(counter)
    used_models_r = used_models_r_n
    print(used_models_r)

    used_models_a_n = used_models_a
    count_a = Counter(used_models_a)
    counter = 0
    for i in range(len(used_models_a)):
        print(used_models_a[i])
        if count_a[used_models_a[i]] >1:
            used_models_a_n[i] = used_models_a[i]+'_'+str(counter)
            counter += 1
        elif count_a[used_models_a[i]] ==1:
            counter = 0
            used_models_a_n[i] = used_models_a[i]+'_'+str(counter)
    used_models_a = used_models_a_n
    print(used_models_a)

    used_models_o_n = used_models_o
    count_o = Counter(used_models_o)
    counter = 0
    for i in range(len(used_models_o)):
        print(used_models_o[i])
        if count_o[used_models_o[i]] >1:
            used_models_o_n[i] = used_models_o[i]+'_'+str(counter)
            counter += 1
        elif count_o[used_models_o[i]] ==1:
            counter = 0
            used_models_o_n[i] = used_models_o[i]+'_'+str(counter)
    used_models_o = used_models_o_n
    print(used_models_o)
    aux = []
    used_models = used_models_r + used_models_o + used_models_a
    if len(used_models) < n_max_i-1:
        aux.append(random.choice(used_models)) #DUPLICATE ONLY ONE MODEL
        df.loc[aux[0][:-2],'n_uses'] += 1
    #print(df.index)
    count_r = Counter(used_models_r)
    count_a = Counter(used_models_a)
    count_o = Counter(used_models_o)
    for i in range(len(aux)):
        if count_r[aux[i]] == 1:
            used_models_r.append(aux[i][:-1]+str(int(aux[i][-1])+1))
        if count_a[aux[i]] == 1:
            used_models_a.append(aux[i][:-1]+str(int(aux[i][-1])+1))
        if count_o[aux[i]] == 1:
            used_models_o.append(aux[i][:-1]+str(int(aux[i][-1])+1))

    used_models = used_models_r + used_models_o + used_models_a
    
    for i in range(len(used_models)):
        if df['type'][used_models[i][:-2]] == 'a':
            used_models[i] = 'human_vehicle_'+str(used_models[i])+'_cam'
        if used_models[i][:6] == 'youbot' or used_models[i][:6] == 'rbkair':
            used_models[i] = str(used_models[i])+'_cam'
    print(f'AQUI OS NOMES ENVIADOS AO AUX_SAVER:{used_models}')

    param_substitutions = {
        'yaml_filename': map_filepath}

    param_substitutions_2 = {
        'yaml_filename': map_restricted_filepath}

    configured_params = RewrittenYaml(
        source_file=os.path.join(get_package_share_directory('datagen_scripts'), 'nav2_params.yaml'),
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True)

    configured_params_2 = RewrittenYaml(
        source_file=os.path.join(get_package_share_directory('datagen_scripts'), 'nav2_params.yaml'),
        root_key='',
        param_rewrites=param_substitutions_2,
        convert_types=True)

    startup_trigger_cmd = Node(
            package='arm_gz_control',
            executable='startup_trigger',
            name='startup_trigger',
            output='screen',
            emulate_tty=True
            )

    finish_trigger_cmd = Node(
            package='arm_gz_control',
            executable='finish_trigger',
            name='finish_trigger',
            output='screen',
            emulate_tty=True
            )

    map_server_cmd = Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            emulate_tty=True,
            parameters=[configured_params])


    filter_mask_server_cmd = Node(
            package='nav2_map_server',
            executable='map_server',
            name='filter_mask_server',
            output='screen',
            emulate_tty=True,
            parameters=[configured_params_2])

    costmap_filter_info_server_cmd = Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='costmap_filter_info_server',
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[configured_params_2])

    map_restricter_cmd = Node(
            package='datagen_scripts',
            executable='map_restricter',
            name='map_restricter',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'cam_x': cam_x},
                        {'cam_y': cam_y},
                        {'cam_yaw': cam_yaw}
                        ])
    
    aux_data_saver_cmd = Node(
            package='datagen_scripts',
            executable='aux_data_saver',
            name='aux_data_saver',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'run_number': int(run_number)},
                        {'save_path': str(launch_params['camera']['save_path'])},
                        {'used_models': used_models},
                        {'save_lidar': bool(launch_params['camera']['save_lidar'])},
                        {'save_depth': bool(launch_params['camera']['save_depth'])}
                        ])
        
    lyfe_cycle_cmd = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_server',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server','filter_mask_server','costmap_filter_info_server']}])

    clock_bridge_cmd = Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    output='screen',
                    arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                               '/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                               '/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image',
                               '/model/dataset_camera/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'                               
                               ],
                    remappings=[('/model/dataset_camera/pose', '/tf')]
                    )

    with open(map_filepath, "r") as f:
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

    grid_map = np.zeros(shape=(map_img.shape[:2]), dtype=np.uint8)


    for i in range(res[0].shape[0]):
        grid_map[res[0][i], res[1][i]] = 255

    max_x = grid_map.shape[1] * map_meta["resolution"] + map_meta["origin"][0]
    max_y = grid_map.shape[0] * map_meta["resolution"] + map_meta["origin"][1]
    map_meta["max_x"] = round(max_x, 2)
    map_meta["max_y"] = round(max_y, 2)

    ##Map restriction
    cam_x_map = math.floor((float(cam_x) - map_meta["origin"][0]) / map_meta["resolution"])
    cam_y_map = math.floor((float(cam_y) - map_meta["origin"][1]) / map_meta["resolution"])

    # because origin in yaml is at bottom left of image
    cam_y_map = grid_map.shape[0] - cam_y_map

    hfov = 120 * math.pi/180 #Camera's horizontal field of view

    angle1 = (float(-cam_yaw) + (hfov/2))
    if angle1 > math.pi:
        angle1 = -2*math.pi + angle1

    angle2 = (float(-cam_yaw) - (hfov/2))
    if angle2 < -math.pi:
        angle2 = (2*math.pi) + angle2

    #camera and restriction area distance
    camera_dist = 1.5 // map_meta["resolution"] 

    #Points near of camera
    point_2_x = cam_x_map + ((camera_dist* math.cos(angle1)))//1
    point_2_y = cam_y_map + ((camera_dist* math.sin(angle1)))//1
    
    point_3_x = cam_x_map + ((camera_dist* math.cos(angle2)))//1
    point_3_y = cam_y_map + ((camera_dist* math.sin(angle2)))//1

    # Arbitrary maximum camera reach
    #points_max_dist = (grid_map.shape[0] + grid_map.shape[1])//2 
    points_max_dist = 10.5 // map_meta["resolution"] 

    #Points far of camera
    point_4_x = point_2_x + ((points_max_dist* math.cos(angle1)))//1
    point_4_y = point_2_y + ((points_max_dist* math.sin(angle1)))//1

    point_5_x = point_3_x + ((points_max_dist* math.cos(angle2)))//1
    point_5_y = point_3_y + ((points_max_dist* math.sin(angle2)))//1

    # Create mask and draw poly restriction with the calculated points on the grid map
    mask = np.ones(shape=(map_img.shape[:2]), dtype=np.uint8) * 255
    polly_points = np.array([ [point_3_x,point_3_y], [point_2_x,point_2_y], [point_4_x,point_4_y], [point_5_x,point_5_y]], np.int)
    cv2.fillConvexPoly(mask, polly_points, 0)
    grid_map = cv2.bitwise_or(grid_map,mask)

    #Initialize world file (actors need to be loaded directly in the world.sdf file in order to spawn and follow target correctly)
    world = os.path.join(resource_path, 'depot_empty.sdf')
    with open(os.path.join(launch_path, 'human_vehicle_model/wrd_bgn_tmpl.sdf')) as f:
        xml_bgn = f.read()

    with open(world, 'w') as launchp:
        launchp.write(xml_bgn)
    
    with open(os.path.join(launch_path, 'launcher_tmpl.ign'), 'w') as launchp:
        launchp.write(r'<?xml version="1.0"?><gz version="1.0"><plugin name="ignition::launch::GazeboFactory" filename="ignition-launch-gazebo-factory">')


    
    model_instances_cmds = []
    for robot in used_models_r:
        print(robot)

        sdf = os.path.join(resource_path, robot[:-2]+'/model.sdf')
        with open(sdf) as f:
            xml = f.read()
        xml = xml.replace('\'', '"')
        xml = xml.replace('<sdf version="1.9">', '')
        #xml = xml.replace('<sdf version="1.8">', '')
        #xml = xml.replace('<sdf version="1.7">', '')
        #xml = xml.replace('<sdf version="1.6">', '')
        #xml = xml.replace('<sdf version="1.10">', '')
        xml = xml.replace('</sdf>', '')
        xml = xml.replace('</model>', '<plugin filename="gz-sim-label-system" name="gz::sim::systems::Label"><label>'+str(df['class_id'][robot[:-2]])+'</label></plugin><plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher"><publish_link_pose>false</publish_link_pose><publish_collision_pose>false</publish_collision_pose><publish_visual_pose>false</publish_visual_pose><publish_nested_model_pose>true</publish_nested_model_pose><use_pose_vector_msg>true</use_pose_vector_msg><static_update_frequency>10</static_update_frequency></plugin></model>')
        xml = xml.replace(robot[:-2], robot)
        #xml = xml.replace('<path>depth_data</path>', '<path>'+str(launch_params['camera']['save_path'])+'/'+str(run_number)+'/instances/'+robot+'/depth</path>')
        xml = xml.replace('<path>segmentation_data/instance_camera</path>', '<path>'+str(launch_params['camera']['save_path'])+'/'+str(run_number)+'/instances/'+robot+'/instance_data</path>')
        xml = xml.replace('model://'+robot, 'model://'+robot[:-2])


        is_valid_pose = False
        while (is_valid_pose == False):

            #RANDOM INITIAL POSES (->string)
            x = round((random.random()*map_meta["origin"][0])-(random.random()*map_meta["origin"][0]), 4)
            y = round((random.random()*map_meta["origin"][1])-(random.random()*map_meta["origin"][1]), 4)
            z = round(random.random(), 4)
            yaw = round((random.random()*-1.570796327) + (random.random()*1.570796327), 4)

            ##Check if its a valid pose in the Grid map
            i_x = math.floor((x - map_meta["origin"][0]) / map_meta["resolution"])
            i_y = math.floor((y - map_meta["origin"][1]) / map_meta["resolution"])
            i_y = grid_map.shape[0] - i_y

            distance = math.ceil(df['radius'][robot[:-2]] / map_meta["resolution"])

            row_start_idx = 0 if i_y - distance < 0 else i_y - distance
            col_start_idx = 0 if i_x - distance < 0 else i_x - distance

            patch = grid_map[row_start_idx : i_y + distance, col_start_idx : i_x + distance]
            obstacles = np.where(patch == 255)
            if len(obstacles[0]) == 0:
                is_valid_pose = True
                # Draw circle with model radius
                model_circle = np.zeros(shape=(map_img.shape[:2]), dtype=np.uint8)
                cv2.circle(model_circle, (i_x, i_y), distance, 255, -1)
                # Update grid map
                grid_map = cv2.bitwise_or(grid_map,model_circle)

        x = float(x)
        y = float(y)
        yaw = float(yaw)
        print(x,y,yaw)


        urdf_file_name = robot[:-2]+'/urdf/'+robot[:-2]+'.urdf'
        urdf = os.path.join(resource_path,urdf_file_name)
        with open(urdf, 'r') as infp:
            robot_desc = infp.read()
        robot_desc = robot_desc.replace(robot[:-2], robot)
        robot_desc = robot_desc.replace('package://'+robot, 'package://'+robot[:-2])


        with open(world, 'a') as launchp:
            launchp.write(xml.replace('<pose>0 0 0 0 0 0</pose>', '<pose>'+str(x)+' '+str(y)+' 0.1 0 0 '+str(yaw)+'</pose>'))


        if (df['has_arm'][robot[:-2]] == 1) or (df['has_arm'][robot[:-2]] == 2):
            # Common moveit2 params for 6dof and 5dof arms
            # Load and reconfigure YAML moveit2 configs
            # SRDF
            with open(os.path.join(get_package_share_directory('datagen_scripts'),robot[:-2]+'.srdf'),'r') as infp:
                robot_srdf = infp.read()
            robot_srdf = robot_srdf.replace(robot[:-2], robot)

            robot_description_semantic = {"robot_description_semantic": robot_srdf}

            # Kinematics
            with open(os.path.join(get_package_share_directory('datagen_scripts'),'kinematics.yaml'), "r") as f:
                kinematics_content = f.read()

            kinematics = yaml.safe_load(kinematics_content)

            # Planning
            with open(os.path.join(get_package_share_directory('datagen_scripts'),'ompl_planning.yaml'), "r") as f:
                ompl_planning_content = f.read()

            ompl_yaml = yaml.safe_load(ompl_planning_content)

            planning = {"move_group": {
                "planning_plugin": "ompl_interface/OMPLPlanner",
                "request_adapters": """default_planner_request_adapters/AddTimeParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
                "start_state_max_bounds_error": 0.1}}
            # Trajectory Execution
            trajectory_execution = {"allow_trajectory_execution": False,
                                    "moveit_manage_controllers": False}

            
            # Planning Scene
            planning_scene_monitor_parameters = {"publish_planning_scene": True,
                                                 "publish_geometry_updates": True,
                                                 "publish_state_updates": True,
                                                 "publish_transforms_updates": True}

            if (df['has_arm'][robot[:-2]] == 1):
                # parameter for 5DOF arm controller 
                joint_names_list=["shoulder_pan_joint","shoulder_lift_joint","elbow_joint",
                                "wrist_1_joint","wrist_2_joint","wrist_3_joint"]
                ign_joint_topics_list=[]
                for joint_name in joint_names_list:
                    ign_joint_topics_list.append('/model/'+robot+'/joint/'+joint_name+'/0/cmd_pos')

                # Joint limits
                with open(os.path.join(get_package_share_directory('datagen_scripts'),'joint_limits.yaml'), "r") as f:
                    joint_limits_content = f.read()
                    joint_limits_yaml = yaml.safe_load(joint_limits_content)

            if (df['has_arm'][robot[:-2]] == 2):
                # parameter for 5DOF arm controller 
                joint_names_list=["arm_joint_1","arm_joint_2","arm_joint_3",
                               "arm_joint_4","arm_joint_5"]
                ign_joint_topics_list=[]
                for joint_name in joint_names_list:
                    ign_joint_topics_list.append('/model/'+robot+'/joint/'+joint_name+'/0/cmd_pos')
                # Joint limits
                with open(os.path.join(get_package_share_directory('datagen_scripts'),'joint_limits_5.yaml'), "r") as f:
                    joint_limits_content = f.read()
                    joint_limits_yaml = yaml.safe_load(joint_limits_content)

            joint_limits = {"robot_description_planning": joint_limits_yaml}


        else:
            ign_joint_topics_list=[]
            joint_names_list=[]
            robot_description_semantic={"robot_description_semantic": ''}
            kinematics=''
            joint_limits=''
            planning=''
            ompl_yaml=''
            trajectory_execution=''
            planning_scene_monitor_parameters=''


        if (df['mode'][robot[:-2]] == 1):
            nav2_param_file = os.path.join(get_package_share_directory('datagen_scripts'), 'nav2_params_omni.yaml')
        elif (df['mode'][robot[:-2]] == 0):
            nav2_param_file = os.path.join(get_package_share_directory('datagen_scripts'), 'nav2_params.yaml')

        robot_param_substitutions = {
            'base_frame_id': [robot,'/base_footprint'],
            'odom_frame_id': [robot,'/odom'],
            'scan_topic': 'front_laser',
            'robot_base_frame': [robot,'/base_footprint'],
            'odom_topic': 'odom',
            'topic': '/'+robot+'/front_laser',
            'local_costmap.local_costmap.ros__parameters.global_frame': [robot, '/odom'],
            'behavior_server.ros__parameters.global_frame': [robot, '/odom'],
            'x': str(x),
            'y': str(y),
            'yaw': str(yaw),
            'robot_radius': str(df['radius'][robot[:-2]]),
            'inflation_radius': str(df['radius'][robot[:-2]] + 0.55)
            }

        robot_configured_params = RewrittenYaml(
            source_file=nav2_param_file,
            root_key=robot,
            param_rewrites=robot_param_substitutions,
            convert_types=True)

        group = GroupAction(
            [
                #Execute bridge node
                PushRosNamespace(
                    namespace=robot),

                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    output='screen',
                    arguments=[
                        '/'+robot+'/front_laser@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                        #'/'+robot+'/rear_laser@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
                        '/'+robot+'/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                        '/'+robot+'/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                        '/model/'+robot+'/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                        '/'+robot+'/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image',
                        '/'+robot+'/panoptic/colored_map@sensor_msgs/msg/Image[gz.msgs.Image',
                        '/'+robot+'/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model'
                        ],
                    parameters=[{'use_sim_time': True}],
                    remappings=[('/'+robot+'/tf', '/tf'),('/model/'+robot+'/pose', '/tf')]),

                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher',
                    output='screen',
                    parameters=[{'use_sim_time': True, 'robot_description': robot_desc}],
                    arguments=[urdf]),

                Node(
                    package='arm_gz_control', 
                    executable='joint_controller',
                    name="joint_controller",
                    parameters=[{"joint_names": joint_names_list},
                            {"gz_joint_topics": ign_joint_topics_list},
                            {"rate":200},
                            {"use_sim_time": True}
                           ],
                    output='screen',
                    condition=(
                        IfCondition(
                            PythonExpression([
                            str(df['has_arm'][robot[:-2]]),
                            " == ",
                            "1", " or ", str(df['has_arm'][robot[:-2]]), " == ", "2",
                        ])))),

                Node(package="moveit_ros_move_group",
                    executable="move_group",
                    name="move_group",
                    output="screen",
                    parameters=[{'robot_description': robot_desc},
                                robot_description_semantic,
                                kinematics,
                                joint_limits,
                                planning,
                                ompl_yaml,
                                trajectory_execution,
                                planning_scene_monitor_parameters,
                                {"use_sim_time": True},
                                {'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'}],
                    condition=(
                        IfCondition(
                            PythonExpression([
                            str(df['has_arm'][robot[:-2]]),
                            " == ",
                            "1", " or ", str(df['has_arm'][robot[:-2]]), " == ", "2",
                        ])))
                ),
                
                Node(
                    package='datagen_scripts', 
                    executable='test_pose_goal',
                    name="test_pose_goal",
                    output='screen',
                    parameters=[{'use_sim_time': True}],
                    condition=(
                        IfCondition(
                            PythonExpression([
                            str(df['has_arm'][robot[:-2]]),
                            " == ",
                            "1",
                        ])))),

                Node(
                    package='datagen_scripts', 
                    executable='test_pose_goal_5',
                    name="test_pose_goal_5",
                    output='screen',
                    parameters=[{'use_sim_time': True}],
                    condition=(
                        IfCondition(
                            PythonExpression([
                            str(df['has_arm'][robot[:-2]]),
                            " == ",
                            "2",
                        ])))),
                Node(
                    package='nav2_amcl',
                    executable='amcl',
                    name='amcl',
                    output='screen',
                    parameters=[robot_configured_params],
                    remappings=[('/'+robot+'/map', '/map')]),

                Node(
                    package='nav2_controller',
                    executable='controller_server',
                    output='screen',
                    parameters=[robot_configured_params],
                    remappings=[('/'+robot+'/map', '/map')]),

                #smoother

                Node(
                    package='nav2_planner',
                    executable='planner_server',
                    name='planner_server',
                    output='screen',
                    parameters=[robot_configured_params],
                    remappings=[('/'+robot+'/map', '/map')]),

                Node(
                    package='nav2_behaviors', 
                    executable='behavior_server',
                    name='behavior_server',
                    output='screen',
                    parameters=[robot_configured_params],
                    remappings=[('/'+robot+'/map', '/map')]),

                Node(
                    package='nav2_bt_navigator',
                    executable='bt_navigator',
                    name='bt_navigator',
                    output='screen',
                    parameters=[robot_configured_params],
                    remappings=[('/'+robot+'/map', '/map')]),

                Node(
                    package='nav2_waypoint_follower',
                    executable='waypoint_follower',
                    name='waypoint_follower',
                    output='screen',
                    parameters=[robot_configured_params],
                    remappings=[('/'+robot+'/map', '/map')]),

                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager',
                    output='screen',
                    parameters=[{'use_sim_time': True},
                                {'autostart': True},
                                {'node_names': ['amcl','controller_server','planner_server','behavior_server','bt_navigator','waypoint_follower']}]),
                                #, 'controller_server', 'planner_server', 'recoveries_server', 'bt_navigator', 'waypoint_follower'
                Node(
                    package='datagen_scripts', 
                    executable='random_goal_handler',
                    name="random_goal_handler",
                    output='screen',
                    parameters=[{'use_sim_time': True},
                                {'model_radius': float(df['radius'][robot[:-2]])}]),

                LogInfo(condition=IfCondition(log_settings), msg=["Launching ", robot]),

            ]
        )
        
        model_instances_cmds.append(group)


    i = 0
    model_a_instances_cmds = []
    for actor in used_models_a:
        human_vehicle_name = 'human_vehicle_'+actor
        print(str(actor))

        
        #sdf = os.path.join(launch_path, 'human_vehicle_model/actor_tmpl.ign')
        sdf = os.path.join(launch_path, 'human_vehicle_model/actor_wrd_tmpl.sdf')
        with open(sdf) as f:
            xml = f.read()

        is_valid_pose = False
        while (is_valid_pose == False):

            #RANDOM INITIAL POSES (->string)
            x = round((random.random()*map_meta["origin"][0])-(random.random()*map_meta["origin"][0]), 4)
            y = round((random.random()*map_meta["origin"][1])-(random.random()*map_meta["origin"][1]), 4)
            z = round(random.random(), 4)
            yaw = round((random.random()*-1.570796327) + (random.random()*1.570796327), 4)

            ##Check if its a valid pose in the Grid map
            i_x = math.floor((x - map_meta["origin"][0]) / map_meta["resolution"])
            i_y = math.floor((y - map_meta["origin"][1]) / map_meta["resolution"])
            i_y = grid_map.shape[0] - i_y

            distance = math.ceil(df['radius'][actor[:-2]] / map_meta["resolution"])

            row_start_idx = 0 if i_y - distance < 0 else i_y - distance
            col_start_idx = 0 if i_x - distance < 0 else i_x - distance

            patch = grid_map[row_start_idx : i_y + distance, col_start_idx : i_x + distance]
            obstacles = np.where(patch == 255)
            if len(obstacles[0]) == 0:
                is_valid_pose = True
                # Draw circle with model radius
                model_circle = np.zeros(shape=(map_img.shape[:2]), dtype=np.uint8)
                cv2.circle(model_circle, (i_x, i_y), distance, 255, -1)
                # Update grid map
                grid_map = cv2.bitwise_or(grid_map,model_circle)

        x = float(x)
        y = float(y)
        yaw = float(yaw)
        print(x,y,yaw)

        xml = xml.replace('human_vehicle', human_vehicle_name)
        xml = xml.replace('actor_name', actor)
        #xml = xml.replace('<path>depth_data</path>', '<path>'+str(launch_params['camera']['save_path'])+'/'+str(run_number)+'/instances/'+human_vehicle_name+'/depth</path>')
        xml = xml.replace('<path>segmentation_data/instance_camera</path>', '<path>'+str(launch_params['camera']['save_path'])+'/'+str(run_number)+'/instances/'+human_vehicle_name+'/instance_data</path>')

        xml = xml.replace('<pose>0 0 0.325 0 0 0</pose>', '<pose>'+str(x)+' '+str(y)+' 0.325 0 0 '+str(yaw)+'</pose>')
        xml = xml.replace('<pose>0 0 1.0 0 0 0</pose>', '<pose>'+str(x)+' '+str(y)+' 0 0 0 0</pose>')
        xml = xml.replace('<uri></uri>', '<uri>model://'+actor[:-2]+'</uri><plugin filename="gz-sim-label-system" name="gz::sim::systems::Label"><label>'+str(df['class_id'][actor[:-2]])+'</label></plugin>')

        with open(world, 'a') as launchp:
            launchp.write(xml)

        #Human Vehicle urdf description
        urdf_file_name = 'human_vehicle_model/human_vehicle_model.urdf'
        urdf = os.path.join(launch_path,urdf_file_name)
        with open(urdf, 'r') as infp:
            hv_desc = infp.read()
        hv_desc = hv_desc.replace('human_vehicle', human_vehicle_name)        
       
        i += 1

        actor_param_substitutions = {
            'base_frame_id': [human_vehicle_name,'/base_footprint'],
            'odom_frame_id': [human_vehicle_name,'/odom'],
            'scan_topic': '/'+human_vehicle_name+'/front_laser',
            'robot_base_frame': [human_vehicle_name,'/base_footprint'],
            'odom_topic': 'odom',
            'topic': '/'+human_vehicle_name+'/front_laser',
            'local_costmap.local_costmap.ros__parameters.global_frame': [human_vehicle_name, '/odom'],
            'behaviour_server.ros__parameters.global_frame': [human_vehicle_name, '/odom'],
            'x': str(x),
            'y': str(y),
            'yaw': str(yaw),
            'robot_radius': str(df['radius'][actor[:-2]]),
            'inflation_radius': str(df['radius'][actor[:-2]] + 0.55)
            }

        actor_configured_params = RewrittenYaml(
            source_file=os.path.join(get_package_share_directory('datagen_scripts'), 'nav2_params.yaml'),
            root_key=human_vehicle_name,
            param_rewrites=actor_param_substitutions,
            convert_types=True)

        group = GroupAction(
            [
                #Execute bridge node
                PushRosNamespace(
                    namespace=human_vehicle_name),

                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    output='screen',
                    arguments=[
                        '/'+human_vehicle_name+'/front_laser@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                        #'/'+robot+'/rear_laser@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
                        '/'+human_vehicle_name+'/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                        '/'+human_vehicle_name+'/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                        '/'+human_vehicle_name+'/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                        '/model/'+human_vehicle_name+'/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                        '/'+human_vehicle_name+'/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image',
                        '/'+human_vehicle_name+'/panoptic/colored_map@sensor_msgs/msg/Image[gz.msgs.Image'
                        ],
                    parameters=[{'use_sim_time': True}],
                    remappings=[('/'+human_vehicle_name+'/tf', '/tf'), ('/model/'+human_vehicle_name+'/pose', '/tf')]),

                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher',
                    output='screen',
                    parameters=[{'use_sim_time': True, 'robot_description': hv_desc}],
                    arguments=[urdf]),

               
                Node(
                    package='nav2_amcl',
                    executable='amcl',
                    name='amcl',
                    output='screen',
                    parameters=[actor_configured_params],
                    remappings=[('/'+human_vehicle_name+'/map', '/map')]),

                Node(
                    package='nav2_controller',
                    executable='controller_server',
                    output='screen',
                    parameters=[actor_configured_params],
                    remappings=[('/'+human_vehicle_name+'/map', '/map')]),

                Node(
                    package='nav2_planner',
                    executable='planner_server',
                    name='planner_server',
                    output='screen',
                    parameters=[actor_configured_params],
                    remappings=[('/'+human_vehicle_name+'/map', '/map')]),

                Node(
                    package='nav2_behaviors', 
                    executable='behavior_server',
                    name='behavior_server',
                    output='screen',
                    parameters=[actor_configured_params],
                    remappings=[('/'+human_vehicle_name+'/map', '/map')]),

                Node(
                    package='nav2_bt_navigator',
                    executable='bt_navigator',
                    name='bt_navigator',
                    output='screen',
                    parameters=[actor_configured_params],
                    remappings=[('/'+human_vehicle_name+'/map', '/map')]),

                Node(
                    package='nav2_waypoint_follower',
                    executable='waypoint_follower',
                    name='waypoint_follower',
                    output='screen',
                    parameters=[actor_configured_params],
                    remappings=[('/'+human_vehicle_name+'/map', '/map')]),

                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager',
                    output='screen',
                    parameters=[{'use_sim_time': True},
                                {'autostart': True},
                                {'node_names': ['amcl','controller_server','planner_server','behavior_server','bt_navigator','waypoint_follower']}]),
                                #, 'controller_server', 'planner_server', 'recoveries_server', 'bt_navigator', 'waypoint_follower'
                Node(
                    package='datagen_scripts', 
                    executable='random_goal_handler',
                    name="random_goal_handler",
                    output='screen',
                    parameters=[{'use_sim_time': True},
                                {'model_radius': float(df['radius'][actor[:-2]])}]),

                LogInfo(condition=IfCondition(log_settings), msg=["Launching ", actor]),

            ]
        )
        
        model_a_instances_cmds.append(group)


    model_o_instances_cmds = []
    for object in used_models_o:

        print(object)

        is_valid_pose = False
        while (is_valid_pose == False):

            #RANDOM INITIAL POSES (->string)
            x = round((random.random()*map_meta["origin"][0])-(random.random()*map_meta["origin"][0]), 4)
            y = round((random.random()*map_meta["origin"][1])-(random.random()*map_meta["origin"][1]), 4)
            z = round(random.random(), 4)
            yaw = round((random.random()*-1.570796327) + (random.random()*1.570796327), 4)

            ##Check if its a valid pose in the Grid map
            i_x = math.floor((x - map_meta["origin"][0]) / map_meta["resolution"])
            i_y = math.floor((y - map_meta["origin"][1]) / map_meta["resolution"])
            i_y = grid_map.shape[0] - i_y

            distance = math.ceil(df['radius'][object[:-2]] / map_meta["resolution"])

            row_start_idx = 0 if i_y - distance < 0 else i_y - distance
            col_start_idx = 0 if i_x - distance < 0 else i_x - distance

            patch = grid_map[row_start_idx : i_y + distance, col_start_idx : i_x + distance]
            obstacles = np.where(patch == 255)
            if len(obstacles[0]) == 0:
                is_valid_pose = True
                # Draw circle with model radius
                model_circle = np.zeros(shape=(map_img.shape[:2]), dtype=np.uint8)
                cv2.circle(model_circle, (i_x, i_y), distance, 255, -1)
                # Update grid map
                grid_map = cv2.bitwise_or(grid_map,model_circle)

        x = float(x)
        y = float(y)
        yaw = float(yaw)
        print(x,y,yaw)

        with open(world, 'a') as launchp:
            launchp.write('<include><name>'+object+'</name><pose>'+str(x)+' '+str(y)+' 0.1 0 0 '+str(yaw)+'</pose><uri>model://'+object[:-2]+'</uri><plugin filename="gz-sim-label-system" name="gz::sim::systems::Label"><label>'+str(df['class_id'][object[:-2]])+'</label></plugin><static>false</static><plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher"><publish_link_pose>false</publish_link_pose><publish_collision_pose>false</publish_collision_pose><publish_visual_pose>false</publish_visual_pose><publish_nested_model_pose>true</publish_nested_model_pose><use_pose_vector_msg>true</use_pose_vector_msg><static_update_frequency>10</static_update_frequency></plugin></include>')

        group = GroupAction(
            [
                #Execute bridge node
                PushRosNamespace(
                    namespace=object),

                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    output='screen',
                    arguments=[
                        '/model/'+object+'/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                        '/'+object+'/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image',
                        '/'+object+'/panoptic/colored_map@sensor_msgs/msg/Image[gz.msgs.Image'
                        ],
                    parameters=[{'use_sim_time': True}],
                    remappings=[('/model/'+object+'/pose', '/tf')]),
                    LogInfo(condition=IfCondition(log_settings), msg=["Launching ", object]),
            ]
        )    
        model_o_instances_cmds.append(group)

    ## Save data frame in CSV format
    df.to_csv(launch_path+'/data_frame_dict.csv', index=True)
    ## Add camera sensors and close world file and create launch_cmd
    camera_sdf = os.path.join(launch_path, 'camera_tmpl.sdf')
    with open(camera_sdf) as f:
        cam_xml = f.read()
    cam_z = float(launch_params['camera']['z'])-0.03585
    cam_xml = cam_xml.replace('<pose>0 0 2 0 -0.523599 0</pose>', '<pose>'+str(cam_x)+' '+str(cam_y)+' '+str(cam_z)+' 0 '+str(launch_params['camera']['pitch'])+' '+str(cam_yaw)+'</pose>')
    
    if launch_params['camera']['instance_seg'] == true:
        cam_xml = cam_xml.replace('<!--instance','')
        cam_xml = cam_xml.replace('instance-->','')
        cam_xml = cam_xml.replace('segmentation_data/instance_camera',  str(launch_params['camera']['save_path'])+'/'+str(run_number)+'/segmentation_data/instance_camera')

    if launch_params['camera']['semantic_seg'] == true:
        cam_xml = cam_xml.replace('<!--semantic','')
        cam_xml = cam_xml.replace('semantic-->','')
        cam_xml = cam_xml.replace('segmentation_data/semantic_camera',  str(launch_params['camera']['save_path'])+'/'+str(run_number)+'/segmentation_data/semantic_camera')

    if launch_params['camera']['bb2d_vis'] == true:
        cam_xml = cam_xml.replace('<!--vis','')
        cam_xml = cam_xml.replace('vis-->','')
        cam_xml = cam_xml.replace('bounding_box_visible_2d_data',  str(launch_params['camera']['save_path'])+'/'+str(run_number)+'/bounding_box_visible_2d_data')

    if launch_params['camera']['bb2d_full'] == true:
        cam_xml = cam_xml.replace('<!--full','')
        cam_xml = cam_xml.replace('full-->','')
        cam_xml = cam_xml.replace('bounding_box_full_2d_data',  str(launch_params['camera']['save_path'])+'/'+str(run_number)+'/bounding_box_full_2d_data')

    if launch_params['camera']['bb3d'] == true:
        cam_xml = cam_xml.replace('<!--3d','')
        cam_xml = cam_xml.replace('3d-->','')
        cam_xml = cam_xml.replace('bounding_box_3d_data',  str(launch_params['camera']['save_path'])+'/'+str(run_number)+'/bounding_box_3d_data')

    if launch_params['camera']['save_depth'] == true:
        cam_xml = cam_xml.replace('<!--depth','')
        cam_xml = cam_xml.replace('depth-->','')
        cam_xml = cam_xml.replace('depth_data', str(launch_params['camera']['save_path'])+'/'+str(run_number)+'/depth_data')

    if launch_params['camera']['save_lidar'] == true:
        cam_xml = cam_xml.replace('<!--lidar','')
        cam_xml = cam_xml.replace('lidar-->','')

    with open(world, 'a') as launchp:
            launchp.write(cam_xml)

    cam_pose_pub_cmd = ExecuteProcess(
        #cmd=['ros2', 'topic', 'pub', '/camera_pose', 'geometry_msgs/msg/Pose', '\'{position: {x: '+str(cam_x)+',y: '+str(cam_y)+'}, orientation: {x: '+str(qx)+', y: '+str(qy)+', z: '+str(qz)+', w: '+str(qw)+'}}\'', '-1'],
        cmd=['ros2', 'topic', 'pub', '/camera_pose', 'geometry_msgs/msg/Pose', '"{position: {x: 0}}"', '-1'],
        output='screen')
    
    launch_world_cmd = ExecuteProcess(
        #cmd=['gz', 'launch', 'depot_world.gzlaunch'],
        cmd=['gz', 'sim', 'depot_empty.sdf', '-r', '-s', '--iterations', str(launch_params['sim_iterations_per_run'])],
        #cmd=['gz', 'sim', 'depot_empty.sdf', '-r', '--iterations', str(launch_params['sim_iterations_per_run'])],
        output='screen')

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(launch_world_cmd)
    ld.add_action(map_server_dcl)
    #ld.add_action(cam_pose_pub_cmd)

    ld.add_action(startup_trigger_cmd)
    ld.add_action(finish_trigger_cmd)

    ld.add_action(RegisterEventHandler(
        OnProcessExit(
            target_action=startup_trigger_cmd,
            on_exit=[
                    LogInfo(msg=("Simulation started. Starting launch nodes...")),

                    lyfe_cycle_cmd,
                    map_server_cmd,
                    filter_mask_server_cmd,
                    costmap_filter_info_server_cmd,
                    map_restricter_cmd,
                    aux_data_saver_cmd,
                    clock_bridge_cmd,
                    #cam_pose_pub_cmd,
                ]
        )
    )

    )
    for simulation_instance_cmd in model_instances_cmds:
        ld.add_action(RegisterEventHandler(
            OnProcessStart(
                target_action=clock_bridge_cmd,
                on_start=[
                        simulation_instance_cmd

                    ]
            )
        )

        )
    for simulation_a_instance_cmd in model_a_instances_cmds:
        ld.add_action(RegisterEventHandler(
            OnProcessStart(
                target_action=clock_bridge_cmd,
                on_start=[
                        simulation_a_instance_cmd

                    ]
            )
        )

        )

    for simulation_o_instance_cmd in model_o_instances_cmds:
        ld.add_action(RegisterEventHandler(
            OnProcessStart(
                target_action=clock_bridge_cmd,
                on_start=[
                        simulation_o_instance_cmd

                    ]
            )
        )

        )

    ld.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=finish_trigger_cmd,
                on_exit=[
                    EmitEvent(event=Shutdown(
                        reason='Simulation iterations completed.'))
                ]
            )
        ),
    )

    return ld
