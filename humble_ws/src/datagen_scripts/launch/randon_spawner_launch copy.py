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
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_common.launch import RewrittenYaml

def Convert(string):
    li = list(string.split(" "))
    return li

def generate_launch_description():
    launch_path=os.environ['IGN_LAUNCH_CONFIG_PATH']
    log_settings = LaunchConfiguration("log_settings", default="true")
    map = os.path.join(get_package_share_directory('datagen_scripts'), 'map.yaml')
    cam_x = 0.0
    cam_y = 0.0
    cam_yaw = 0.0
    #TO DO: PUT THAT LIST IN A FILE, yaml preferably
    #models = ['rbkairos_migration_sensor_config_1','rbkairos_migration_sensor_config_2','P3-dx','p3-at','kukka','donker','human1','human2','human3','human4','human5','human6']
    models = [] #List of Robots
    models_a = [] #List of Actors
    models_o = [] #List of Objects

    used_models_r = [] #List of used models in this run
    used_models_a = []
    used_models_o = []

    n_max_r = 2 #Max of robots to spawn
    n_max_a = 0
    n_max_o = 0

    random.seed(15)

    map_server_dcl = DeclareLaunchArgument(
            'map',
            default_value=os.path.join(get_package_share_directory('datagen_scripts'), 'map.yaml'),
            description='Full path to map yaml file to load')

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
        'yaml_filename': os.path.join(get_package_share_directory('datagen_scripts'), 'map.yaml')}

    configured_params = RewrittenYaml(
        source_file=os.path.join(get_package_share_directory('nav2_bringup'), 'params', 'nav2_params.yaml'),
        param_rewrites=param_substitutions,
        convert_types=True)



    map_server_cmd = Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[configured_params])

    map_restricter_cmd = Node(
            package='datagen_scripts',
            executable='map_restricter',
            name='map_restricter',
            output='screen',
            parameters=[{'use_sim_time': True}])
        
    ##lyfe_cycle_cmd = Node(
    ##        package='nav2_lifecycle_manager',
    ##        executable='lifecycle_manager',
    ##        name='lifecycle_manager_map_server',
    ##        output='screen',
    ##        parameters=[{'use_sim_time': True},
    ##                    {'autostart': True},
    ##                    {'node_names': ['map_server','map_restricter']}])


    with open(map, "r") as f:
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
    print(df['type']['rbkairos_migration_sensor_config_1'])

    #Fill String lists
    for name in df.index:
        if df['type'][name] == 'r':
            models.append(name)
        elif df['type'][name] == 'a':
            models_a.append(name)
        elif df['type'][name] == 'o':
            models_o.append(name)


    with open(os.path.join(launch_path, 'launcher_tmpl.ign'), 'w') as launchp:
        #content = launchp.read()
        #print(content)
        launchp.write(r'<?xml version="1.0"?><ignition version="1.0"><plugin name="ignition::launch::GazeboFactory" filename="ignition-launch-gazebo-factory">')

    while len(used_models_r) < n_max_r:
        i=0
        curr_model=random.choice(models)
        curr_model_name = curr_model + '_'+ str(i)

        while curr_model_name in used_models_r:
            i += 1
            curr_model_name = curr_model + '_'+ str(i)    
        used_models_r.append(curr_model_name)
    
    #print(used_models_r)
    # Define commands for spawning models and launching the model instances
    model_instances_cmds = []
    for robot in used_models_r:
        print(robot)
      #  sdf = os.path.join(get_package_share_directory(curr_model), 'model.sdf')
      #  with open(sdf) as f:
      #      xml = f.read()
      #  xml = xml.replace(curr_model, robot)
      #  #FIX ABSOLUTE PATH
      #  xml = xml.replace('meshes', os.path.join(get_package_share_directory(curr_model), robot, 'meshes'))
      #  xml = xml.replace('materials', os.path.join(get_package_share_directory(curr_model), robot, 'materials'))
      #  xml = xml.replace('"', '\\"')
      #  xml = xml.replace('\n', ' ')

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

        with open(os.path.join(launch_path, 'launcher_tmpl.ign'), 'a') as launchp:
            launchp.write('<spawn><name>'+robot+'</name><allow_renaming>true</allow_renaming><pose>'+str(x)+' '+str(y)+' '+str(z)+' 0 0 '+str(yaw)+'</pose>'+xml+'</spawn>')
            #ign_launch = ign_launch.replace('<!--key-->','<spawn><name>'+robot+'</name><allow_renaming>true</allow_renaming><pose>'+x+' '+y+' '+z+' 0 0 '+yaw+'</pose><sdf version="1.9"><include><uri></uri></include></sdf></spawn>')

        #params_file = LaunchConfiguration(robot + "_params_file")

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
                        '/'+robot+'/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
                        '/'+robot+'/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock'
                        ],
                    remappings=[('/'+robot+'/tf', '/tf')]),

                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher',
                    output='screen',
                    parameters=[{'use_sim_time': True, 'robot_description': robot_desc}],
                    arguments=[urdf]),
                
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


#    while len(used_models_a) < n_max_a:
#        i=0
#        curr_model=random.choice(models)
#        curr_model_name = curr_model + '_'+ str(i)
#
#        while curr_model_name in used_models_a:
#            i += 1
#            curr_model_name = curr_model + '_'+ str(i)    
#        used_models_a.append(curr_model_name)


    while len(used_models_o) < n_max_o:
        i=0
        curr_model=random.choice(models)
        curr_model_name = curr_model + '_'+ str(i)

        while curr_model_name in used_models_o:
            i += 1
            curr_model_name = curr_model + '_'+ str(i)    
        used_models_o.append(curr_model_name)

    for object in used_models_o:

        #RANDOM INITIAL POSES (->string)
        x = str((random.random()*5.0000)-(random.random()*5.0000))
        y = str((random.random()*5.0000)-(random.random()*5.0000))
        z = str(random.random())
        yaw = str((random.random()*-1,570796327) + (random.random()*1,570796327))

    
    ##Close launch file and create launch_cmd
    with open(os.path.join(launch_path, 'launcher_tmpl.ign'), 'a') as launchp:
        launchp.write('</plugin></ignition>')
    
    launch_cmd = ExecuteProcess(
        cmd=['ign', 'launch', 'launcher_tmpl.ign'],
        output='screen')

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(map_server_dcl)
    ld.add_action(cam_x_dcl)
    ld.add_action(cam_y_dcl)
    ld.add_action(cam_yaw_dcl)

    ld.add_action(map_server_cmd)
    ld.add_action(map_restricter_cmd)
    #ld.add_action(lyfe_cycle_cmd)
    ld.add_action(launch_cmd)

    for simulation_instance_cmd in model_instances_cmds:
        ld.add_action(simulation_instance_cmd)

    return ld

#
#
#    sdf = os.path.join(get_package_share_directory(curr_model), 'model.sdf')
#    with open(sdf) as f:
#        xml = f.read()
#    xml = xml.replace(curr_model, curr_model_name)
#    #FIX ABSOLUTE PATH
#    xml = xml.replace('meshes', '/home/danilo/galactic_ws/src/ign_models/'+curr_model+'/meshes')
#    xml = xml.replace('materials', '/home/danilo/galactic_ws/src/ign_models/'+curr_model+'/materials')
#    xml = xml.replace('"', '\\"')
#    xml = xml.replace('\n', ' ')
#    #xml = xml.encode('unicode-escape').decode()
#    spawn_args = r'sdf: "'+ xml + r'" pose: {position: {z: 1}} name: "rb" allow_renaming: true'
#
#    return LaunchDescription([
#        ExecuteProcess(
#            cmd=['ign', 'service', '-s', '/world/empty/create','--reqtype','ignition.msgs.EntityFactory','--reptype','ignition.msgs.Boolean','--timeout','300','--req', spawn_args],
#            output='screen')
#    ])
#
#    r'sdf: "<?xml version=\"1.0\"?><sdf version=\"1.6\"><model name=\"spawned_model\"><link name=\"link\"><visual name=\"visual\"><geometry><sphere><radius>1.0</radius></sphere></geometry></visual><collision name=\"visual\"><geometry><sphere><radius>1.0</radius></sphere></geometry></collision></link></model></sdf>" pose: {position: {z: 10}} name: "new_name" allow_renaming: true'