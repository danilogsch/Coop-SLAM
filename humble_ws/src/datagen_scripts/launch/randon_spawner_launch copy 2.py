from ast import Not
from distutils.spawn import spawn
import os
import random
import pandas
import yaml
from PIL import Image
import numpy as np
import math
import cv2

from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription, LogInfo
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_common.launch import RewrittenYaml

def Convert(string):
    li = list(string.split(" "))
    return li

def generate_launch_description():

    map_server_dcl = DeclareLaunchArgument(
            'map',
            default_value=os.path.join(get_package_share_directory('datagen_scripts'), 'map.yaml'),
            description='Full path to map yaml file to load')
    launch_path=os.environ['IGN_LAUNCH_CONFIG_PATH']
    resource_path = os.environ['IGN_GAZEBO_RESOURCE_PATH'].split(':')
    resource_path = resource_path[-1]

    log_settings = LaunchConfiguration("log_settings", default="true")

    map_restricted_filepath = os.path.join(get_package_share_directory('datagen_scripts'), 'map_restricted.yaml')
    map_filepath = os.path.join(get_package_share_directory('datagen_scripts'), 'map.yaml')

    cam_x = 0.0
    cam_y = 0.0
    cam_yaw = 0.0

    models = [] #List of Robots
    models_a = [] #List of Actors
    models_o = [] #List of Objects

    used_models_r = [] #List of used models in this run
    used_models_a = []
    used_models_o = []

    n_max_r = 2 #Max of robots to spawn
    n_max_a = 2
    n_max_o = 2

    random.seed(638764434145371)

    cam_x_dcl = DeclareLaunchArgument(
            'cam_x',
            default_value= '0.0',
            description='Segmentation/BB Camera x position in meters.')

    cam_y_dcl = DeclareLaunchArgument(
            'cam_y',
            default_value= '0.0',
            description='Segmentation/BB Camera y position in meters.')

    cam_yaw_dcl = DeclareLaunchArgument(
            'cam_yaw',
            default_value= '0.0',
            description='Segmentation/BB Camera yaw direction in rad.')

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
            parameters=[{'use_sim_time': True}])
        
    lyfe_cycle_cmd = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_server',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server','filter_mask_server','costmap_filter_info_server']}])

#    lyfe_cycle_cmd_2 = Node(
#            package='nav2_lifecycle_manager',
#            executable='lifecycle_manager',
#            name='lifecycle_manager_costmap_filters',
#            output='screen',
#            emulate_tty=True,
#            parameters=[{'use_sim_time': True},
#                        {'autostart': True},
#                        {'node_names': ['filter_mask_server','costmap_filter_info_server']}])

    clock_bridge_cmd = Node(
                    package='ros_ign_bridge',
                    executable='parameter_bridge',
                    output='screen',
                    arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock']
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

    hfov = 90 * math.pi/180 #Camera's horizontal field of view

    angle1 = (float(cam_yaw) + (hfov/2))
    if angle1 > math.pi:
        angle1 = -2*math.pi + angle1

    angle2 = (float(cam_yaw) - (hfov/2))
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
    points_max_dist = (grid_map.shape[0] + grid_map.shape[1])//2 

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


    
    #Read DataFrame file:
    df = pandas.read_csv(launch_path+'/data_frame_dict.csv', 
            index_col='Name')
    #print(df['type']['rbkairos_migration_sensor_config_1'])
    df = df.sort_values(
        by=['type','n_uses']
        )

    #Fill String lists
    for name in df.index:
        if df['type'][name] == 'r':
            models.append(name)
        elif df['type'][name] == 'a':
            models_a.append(name)
        elif df['type'][name] == 'o':
            models_o.append(name)

    if len(models) > n_max_r:
        models = models[:n_max_r]

    if len(models_a) > n_max_a:
        models_a = models_a[:n_max_a]
    print (models_a)
    if len(models_o) > n_max_o:
        models_o = models_o[:n_max_o]
    

    with open(os.path.join(launch_path, 'launcher_tmpl.ign'), 'w') as launchp:
        launchp.write(r'<?xml version="1.0"?><ignition version="1.0"><plugin name="ignition::launch::GazeboFactory" filename="ignition-launch-gazebo-factory">')

    while len(used_models_r) < n_max_r:
        i=0
        curr_model=random.choice(models)
        curr_model_name = curr_model + '_'+ str(i)

        while curr_model_name in used_models_r:
            i += 1
            curr_model_name = curr_model + '_'+ str(i)    
        used_models_r.append(curr_model_name)
    
    model_instances_cmds = []
    for robot in used_models_r:
        print(robot)

        sdf = os.path.join(get_package_share_directory(robot[:-2]), 'model.sdf')
        with open(sdf) as f:
            xml = f.read()
        xml = xml.replace(robot[:-2], robot)
        xml = xml.replace('model://'+robot, 'model://'+robot[:-2])
        #FIX ABSOLUTE PATH
        #xml = xml.replace('meshes', os.path.join(get_package_share_directory(curr_model), 'meshes'))
        #xml = xml.replace('materials', os.path.join(get_package_share_directory(curr_model), 'materials'))

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

        ##xml = xml.encode('unicode-escape').decode()
        #spawn_args = r'sdf: "'+ xml + r'" pose: {position: {x: '+x+r', y: '+y+r', z: '+z+r'}, orientation: {yaw:'+yaw+r'}} name: "'+robot+r'" allow_renaming: true'

        #bridge = os.path.join(get_package_share_directory(robot[-2]), 'bridge_topics.txt')
        #with open(bridge) as f:
        #    bridge_topics = f.read() #.encode('unicode-escape').decode()
        #bridge_topics = bridge_topics.replace(robot[-2], robot)
        #bridge_topics = bridge_topics.strip()
        ##bridge_topics = bridge_topics.replace('', robot)

        urdf_file_name = robot[:-2]+'.urdf'
        urdf = os.path.join(get_package_share_directory(robot[:-2]),urdf_file_name)
        with open(urdf, 'r') as infp:
            robot_desc = infp.read()
        robot_desc = robot_desc.replace(robot[:-2], robot)
        robot_desc = robot_desc.replace('package://'+robot, 'package://'+robot[:-2])

        with open(os.path.join(launch_path, 'launcher_tmpl.ign'), 'a') as launchp:
            launchp.write('<spawn><name>'+robot+'</name><allow_renaming>true</allow_renaming><pose>'+str(x)+' '+str(y)+' '+str(z)+' 0 0 '+str(yaw)+'</pose>'+xml+'</spawn>')
            #ign_launch = ign_launch.replace('<!--key-->','<spawn><name>'+robot+'</name><allow_renaming>true</allow_renaming><pose>'+x+' '+y+' '+z+' 0 0 '+yaw+'</pose><sdf version="1.9"><include><uri></uri></include></sdf></spawn>')

        #params_file = LaunchConfiguration(robot + "_params_file")
        if (df['has_arm'][robot[:-2]] == 1):
            # parameter for arm controller
            joint_names_list=["shoulder_pan_joint","shoulder_lift_joint","elbow_joint",
                        "wrist_1_joint","wrist_2_joint","wrist_3_joint"]
            ign_joint_topics_list=[]
            for joint_name in joint_names_list:
                ign_joint_topics_list.append('/model/'+robot+'/joint/'+joint_name+'/0/cmd_pos')

            #Load and reconfigure YAML moveit2 configs
            # SRDF
            with open(os.path.join(get_package_share_directory('datagen_scripts'),robot[:-2]+'.srdf'),'r') as infp:
                robot_srdf = infp.read()
            robot_srdf = robot_srdf.replace(robot[:-2], robot)

            robot_description_semantic = {"robot_description_semantic": robot_srdf}

            # Kinematics
            with open(os.path.join(get_package_share_directory('datagen_scripts'),'kinematics.yaml'), "r") as f:
                kinematics_content = f.read()

            kinematics = yaml.safe_load(kinematics_content)

            # Joint limits
            with open(os.path.join(get_package_share_directory('datagen_scripts'),'joint_limits.yaml'), "r") as f:
                joint_limits_content = f.read()
                joint_limits_yaml = yaml.safe_load(joint_limits_content)

            joint_limits = {"robot_description_planning": joint_limits_yaml}

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

        # Nav Parameters:
        #with open(os.path.join(get_package_share_directory('datagen_scripts'),'nav2_params.yaml'), "r") as f:
        #    joint_limits_content = f.read()
        #with open(os.path.join(get_package_share_directory('datagen_scripts'),'nav2_params.yaml'), "r") as f:
        #    joint_limits_content = f.read()
        #    joint_limits_yaml = yaml.safe_load(joint_limits_content)
        robot_param_substitutions = {
            'base_frame_id': [robot,'/base_footprint'],
            'odom_frame_id': [robot,'/odom'],
            'scan_topic': 'front_laser',
            'robot_base_frame': [robot,'/base_footprint'],
            'odom_topic': 'odom',
            'topic': '/'+robot+'/front_laser',
            'local_costmap.local_costmap.ros__parameters.global_frame': [robot, '/odom'],
            'recoveries_server.ros__parameters.global_frame': [robot, '/odom'],
            'x': str(x),
            'y': str(y),
            'yaw': str(yaw),
            'robot_radius': str(df['radius'][robot[:-2]]),
            'inflation_radius': str(df['radius'][robot[:-2]])
            }

        robot_configured_params = RewrittenYaml(
            source_file=os.path.join(get_package_share_directory('datagen_scripts'), 'nav2_params_omni.yaml'),
            root_key=robot,
            param_rewrites=robot_param_substitutions,
            convert_types=True)

        group = GroupAction(
            [
                ###to do: RANDOM POSE GENERATION
                #SPAWN MODELS ON GZ WITH NAMESPACE
                #ExecuteProcess(
                #    cmd=['ign', 'service', '-s', '/world/depot/create','--reqtype','ignition.msgs.EntityFactory','--reptype','ignition.msgs.Boolean','--timeout','300','--req', spawn_args],
                #    output='screen'),

                #Execute bridge node
                PushRosNamespace(
                    namespace=robot),

                Node(
                    package='ros_ign_bridge',
                    executable='parameter_bridge',
                    output='screen',
                    arguments=[
                        '/'+robot+'/front_laser@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
                        #'/'+robot+'/rear_laser@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
                        '/'+robot+'/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                        '/'+robot+'/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
                        '/'+robot+'/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model'
                        ],
                    parameters=[{'use_sim_time': True}],
                    remappings=[('/'+robot+'/tf', '/tf')]),

                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher',
                    output='screen',
                    parameters=[{'use_sim_time': True, 'robot_description': robot_desc}],
                    arguments=[urdf]),

                Node(
                    package='universal_robot_ign', 
                    executable='joint_controller',
                    name="ur10_joint_controller",
                    parameters=[{"joint_names": joint_names_list},
                            {"ign_joint_topics": ign_joint_topics_list},
                            {"rate":200},
                            {"use_sim_time": True}
                           ],
                    output='screen',
                    condition=(
                        IfCondition(
                            PythonExpression([
                            str(df['has_arm'][robot[:-2]]),
                            " == ",
                            "1",
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
                                {"use_sim_time": True}],
                    condition=(
                        IfCondition(
                            PythonExpression([
                            str(df['has_arm'][robot[:-2]]),
                            " == ",
                            "1",
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

                Node(
                    package='nav2_planner',
                    executable='planner_server',
                    name='planner_server',
                    output='screen',
                    parameters=[robot_configured_params],
                    remappings=[('/'+robot+'/map', '/map')]),

                Node(
                    package='nav2_recoveries', 
                    executable='recoveries_server',
                    name='recoveries_server',
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
                                {'node_names': ['amcl','controller_server','planner_server','recoveries_server','bt_navigator','waypoint_follower']}]),
                                #, 'controller_server', 'planner_server', 'recoveries_server', 'bt_navigator', 'waypoint_follower'
                Node(
                    package='datagen_scripts', 
                    executable='random_goal_handler',
                    name="random_goal_handler",
                    output='screen',
                    parameters=[{'use_sim_time': True},
                                {'model_radius': float(df['radius'][robot[:-2]])}]),

                LogInfo(condition=IfCondition(log_settings), msg=["Launching ", robot]),

                #LogInfo(
                #    condition=IfCondition(log_settings),
                #    msg=[robot, " SDF: ", xml],
                #),
                #LogInfo(
                #    condition=IfCondition(log_settings), msg=[robot, " URDF: ", robot_desc]
                #)
            ]
        )
        
        model_instances_cmds.append(group)


    while len(used_models_a) < n_max_a:
        i=0
        curr_model=random.choice(models_a)
        curr_model_name = curr_model + '_'+ str(i)

        while curr_model_name in used_models_a:
            i += 1
            curr_model_name = curr_model + '_'+ str(i)    
        used_models_a.append(curr_model_name)

    #Initialize world file (actors need to be loaded directly in the world.sdf file in order to spawn and follow target correctly)
    world = os.path.join(resource_path, 'depot_empty.sdf')
    with open(os.path.join(launch_path, 'human_vehicle_model/wrd_bgn_tmpl.sdf')) as f:
        xml_bgn = f.read()

    with open(world, 'w') as launchp:
        launchp.write(xml_bgn)

    i = 0
    model_a_instances_cmds = []
    for actor in used_models_a:
        human_vehicle_name = 'human_vehicle_'+str(i)
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
        xml = xml.replace('<pose>0 0 0.325 0 0 0</pose>', '<pose>'+str(x)+' '+str(y)+' 0.325 0 0 '+str(yaw)+'</pose>')
        xml = xml.replace('<pose>0 0 1.0 0 0 0</pose>', '<pose>'+str(x)+' '+str(y)+' 0 0 0 0</pose>')
        xml = xml.replace('<uri></uri>', '<uri>model://'+actor[:-2]+'</uri>')

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
            'recoveries_server.ros__parameters.global_frame': [human_vehicle_name, '/odom'],
            'x': str(x),
            'y': str(y),
            'yaw': str(yaw),
            'robot_radius': str(df['radius'][actor[:-2]]),
            'inflation_radius': str(df['radius'][actor[:-2]])
            }

        actor_configured_params = RewrittenYaml(
            source_file=os.path.join(get_package_share_directory('datagen_scripts'), 'nav2_params.yaml'),
            root_key=human_vehicle_name,
            param_rewrites=actor_param_substitutions,
            convert_types=True)

        group = GroupAction(
            [
                ###to do: RANDOM POSE GENERATION
                #SPAWN MODELS ON GZ WITH NAMESPACE
                #ExecuteProcess(
                #    cmd=['ign', 'service', '-s', '/world/depot/create','--reqtype','ignition.msgs.EntityFactory','--reptype','ignition.msgs.Boolean','--timeout','300','--req', spawn_args],
                #    output='screen'),

                #Execute bridge node
                PushRosNamespace(
                    namespace=human_vehicle_name),

                Node(
                    package='ros_ign_bridge',
                    executable='parameter_bridge',
                    output='screen',
                    arguments=[
                        '/'+human_vehicle_name+'/front_laser@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
                        #'/'+robot+'/rear_laser@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
                        '/'+human_vehicle_name+'/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                        '/'+human_vehicle_name+'/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
                        '/'+human_vehicle_name+'/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model'
                        ],
                    parameters=[{'use_sim_time': True}],
                    remappings=[('/'+human_vehicle_name+'/tf', '/tf')]),

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
                    package='nav2_recoveries', 
                    executable='recoveries_server',
                    name='recoveries_server',
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
                                {'node_names': ['amcl','controller_server','planner_server','recoveries_server','bt_navigator','waypoint_follower']}]),
                                #, 'controller_server', 'planner_server', 'recoveries_server', 'bt_navigator', 'waypoint_follower'
                Node(
                    package='datagen_scripts', 
                    executable='random_goal_handler',
                    name="random_goal_handler",
                    output='screen',
                    parameters=[{'use_sim_time': True},
                                {'model_radius': float(df['radius'][actor[:-2]])}]),

                LogInfo(condition=IfCondition(log_settings), msg=["Launching ", actor]),

                #LogInfo(
                #    condition=IfCondition(log_settings),
                #    msg=[robot, " SDF: ", xml],
                #),
                #LogInfo(
                #    condition=IfCondition(log_settings), msg=[robot, " URDF: ", robot_desc]
                #)
            ]
        )
        
        model_a_instances_cmds.append(group)





    while len(used_models_o) < n_max_o:
        i=0
        curr_model=random.choice(models_o)
        curr_model_name = curr_model + '_'+ str(i)

        while curr_model_name in used_models_o:
            i += 1
            curr_model_name = curr_model + '_'+ str(i)    
        used_models_o.append(curr_model_name)

    for object in used_models_o:

        print(object)
        #sdf = os.path.join(resource_path, object[:-2]+'/model.sdf')
        #with open(sdf) as f:
        #    xml = f.read()

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

        

        with open(os.path.join(launch_path, 'launcher_tmpl.ign'), 'a') as launchp:
            launchp.write('<spawn><name>'+object+'</name><allow_renaming>true</allow_renaming><pose>'+str(x)+' '+str(y)+' 0.4 0 0 '+str(yaw)+'</pose><sdf version="1.6"><include><uri>model://'+object[:-2]+'</uri><static>false</static></include></sdf></spawn>')

    ##Close world file and create launch_cmd
    with open(world, 'a') as launchp:
            launchp.write('</world></sdf>')
    ##Close launch file and create launch_cmd
    with open(os.path.join(launch_path, 'launcher_tmpl.ign'), 'a') as launchp:
        launchp.write('</plugin></ignition>')
    
    launch_world_cmd = ExecuteProcess(
        cmd=['ign', 'launch', 'depot_world.ign'],
        output='screen')


    launch_cmd = ExecuteProcess(
        cmd=['ign', 'launch', 'launcher_tmpl.ign'],
        output='screen')

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(launch_world_cmd)

    ld.add_action(map_server_dcl)
    ld.add_action(cam_x_dcl)
    ld.add_action(cam_y_dcl)
    ld.add_action(cam_yaw_dcl)


    ld.add_action(lyfe_cycle_cmd)
    #ld.add_action(lyfe_cycle_cmd_2)

    ld.add_action(map_server_cmd)
    ld.add_action(filter_mask_server_cmd)
    ld.add_action(costmap_filter_info_server_cmd)
    ld.add_action(map_restricter_cmd)
    
    ld.add_action(launch_cmd)
    ld.add_action(clock_bridge_cmd)

    for simulation_instance_cmd in model_instances_cmds:
        ld.add_action(simulation_instance_cmd)

    for simulation_a_instance_cmd in model_a_instances_cmds:
        ld.add_action(simulation_a_instance_cmd)

    return ld
