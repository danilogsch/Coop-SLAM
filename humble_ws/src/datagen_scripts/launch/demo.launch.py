import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from nav2_common.launch import RewrittenYaml

def generate_launch_description():

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    base_frame = LaunchConfiguration('base_frame')
    odom_frame = LaunchConfiguration('odom_frame')
    global_frame = LaunchConfiguration('global_frame')
    scan_topic = LaunchConfiguration('scan_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    lifecycle_nodes = ['map_server', 'amcl']
    bringup_dir = get_package_share_directory('datagen_scripts')
    
    
    urdf_file_name = 'rbkairos.urdf'
    urdf = os.path.join(
        get_package_share_directory('datagen_scripts'),
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()
        
    remappings = []
        
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file,
        'base_frame_id': base_frame,
        'global_frame_id': global_frame,
        'global_frame': global_frame,
        'odom_frame_id': odom_frame,
        'scan_topic': scan_topic,
        'robot_base_frame': base_frame,
        'odom_topic': odom_frame,
        'topic': scan_topic}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)


    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument(
            'namespace', default_value='rbkairos',
            description='Top-level namespace'),

        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(bringup_dir, 'map.yaml'),
            description='Full path to map yaml file to load'),

        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack'),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(get_package_share_directory('nav2_bringup'), 'params', 'nav2_params.yaml'),
            description='Full path to the ROS2 parameters file to use'),
            
        DeclareLaunchArgument(
            'base_frame', default_value='rbkairos/summit_xl_base_footprint',
            description='Top-level namespace'),

        DeclareLaunchArgument(
            'odom_frame', default_value='rbkairos/odom',
            description='Top-level namespace'),
            
        DeclareLaunchArgument(
            'scan_topic', default_value='/rbkairos/front_laser',
            description='Top-level namespace'),
            
        DeclareLaunchArgument(
            'global_frame', default_value='map',
            description='Top-level namespace'),
            
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf]),
            
        PushRosNamespace(
            namespace=namespace),
                        
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[configured_params],
            remappings=remappings),
            
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[configured_params],
            remappings=remappings),
        
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])
    ])

