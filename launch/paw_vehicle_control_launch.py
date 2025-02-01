
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    ld = LaunchDescription()  

    ros_domain_id = os.environ.get('ROS_DOMAIN_ID')

    #vehicle control - manual intervention for autonomous driving
    joy_config = os.path.join(
        get_package_share_directory('vehicle_control'),
        'config',
        'gamepad.yaml'
        )
    joy_config_dict = yaml.safe_load(open(joy_config, 'r'))
    vehicle_control_node = Node(
        package='vehicle_control',
        executable='paw_vehicle_control',
        output='screen',
        name='paw_vehicle_control',
        parameters=[joy_config_dict,{
                    'car_id' : ros_domain_id
                    }]
    )

    #driver for the gamepad
    joy_linux_node = Node(
        package="joy_linux",
        executable="joy_linux_node",
        name="joy_linux_node",
        emulate_tty="true"
    )
     #driver for the gamepad
    gap_follow = Node(
        package="gap_follow",
        executable="gap_follow",
        name="gap_follow"
    )
    
    # pure pursuit race track follower
    race_track_path = os.path.join(
        get_package_share_directory('pure_pursuit'),
        'config'
        )
    
    pure_pursuit_node = Node(
        package="pure_pursuit",
        executable="pure_pursuit",
        name="pure_pursuit",
        output="screen",
        parameters=[{
                     'race_line_csv':race_track_path + '/raceline_v1.csv',
                     'drive_topic': r'/to_drive',
                     'odom_topic':r'odom',
                     'odom_frame': r'ego_racecar/odom'
                     }]
    )
    
    
    ##########################
    # Map server stuff
    #############################
    map_server_config = os.path.join(
        get_package_share_directory('vehicle_control'),
        'config',
        'map_server_params.yaml'
    )
    map_path = os.path.join(
        get_package_share_directory('vehicle_control'),
        'maps',
        'race_track_180624_1300.yaml'
    )
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        parameters=[map_server_config, {
            'yaml_filename': map_path
        }]
    )
    
    # AMCL Node
    amcl_config = os.path.join(
        get_package_share_directory('vehicle_control'),
        'config',
        'amcl_params.yaml'
    )
    # AMCL Node
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_config],
        remappings=[
            ('/map', '/map')
        ]
    )
    """
    # SLAM Toolbox
    slam_toolbox_config = os.path.join(
        get_package_share_directory('vehicle_control'),
        'config',
        'mapper_params_online_async.yaml'
    )
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_toolbox_config],
        remappings=[
            ('/scan', '/scan')
        ]
    )
    """
    
    # manage the nav2 stack nodes: map server, amcl
    nav_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['map_server', 'amcl']}] # 'nav2_costmap_2d',
    )
    
    #driver for lidar
    lidar_launchfile = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([
                            FindPackageShare("rplidar_ros"), '/launch', '/rplidar_s3_launch.py'])
                        )

    #driver for vesc
    vesc_launchfile = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([
                            FindPackageShare("vesc_driver"), '/launch', '/vesc_driver_node.launch.py'])
                        )

    # Check environment variable ROS_DOMAIN_ID and assign odom parameter launch file for ID
    if ros_domain_id == "26": 
        vesc_to_odom_node_launch_path = '/vesc_to_odom_node_racecar_26.launch.xml'
    else: 
        vesc_to_odom_node_launch_path = '/vesc_to_odom_node_racecar_25.launch.xml'
        
    print("using '{}' as configuration file for VESC ".format(vesc_to_odom_node_launch_path), flush=True)

    vesc_odometer = IncludeLaunchDescription(
                        AnyLaunchDescriptionSource([
                            FindPackageShare("vesc_ackermann"), '/launch', vesc_to_odom_node_launch_path])
                        )

    #ackermann driver for vesc TODO: check if those are even used, or only the ones hardcoded into the vesc_ackermann launch files
    #but instead of listening to /ackermann_cmd, we want to listen to /drive for input messages
     # Check environment variable ROS_DOMAIN_ID and assign parameter for ID
    if ros_domain_id == "26": # Parameter for racecar 26
        ackermann_parameters=[{'speed_to_erpm_gain' : 5038.0}, 
                    {'speed_to_erpm_offset': 0.0},
                    {'steering_angle_to_servo_gain': -0.781},
                    {'steering_angle_to_servo_offset': 0.5004},
                   ]
        
    else: # Parameter for racecar 25
        ackermann_parameters=[{'speed_to_erpm_gain' : 6254.0}, #was 5254 with paw tires
                    {'speed_to_erpm_offset': 0.0},
                    {'steering_angle_to_servo_gain': -0.702},  #was -0.752 with paw tires
                    {'steering_angle_to_servo_offset': 0.515}, #was 0.525 with paw tires
                   ]
    
    vesc_ackermann = Node(
        package="vesc_ackermann",
        executable="ackermann_to_vesc_node",
        name="ackermann_to_vesc_node",
        parameters=ackermann_parameters,
        remappings=[('/ackermann_cmd', '/drive')] 
    ) 
    


    #transforms
    transform_imu_baselink = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '0.23', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'ego_racecar/base_link', '--child-frame-id', 'imu']
        )

    emergency_braking_node = Node(
        package='emergency_braking',
        executable='emergency_braking',
        output='screen',
        name='emergency_braking',
        parameters=[]
    )
    
    camera_node = Node(
        package='camera_ros',
        executable='camera_node',
        output='screen',
        name='camera',
        parameters=[
            {'width': 1152},
            {'height': 648},
            {'format': 'XRGB8888'}]
    )

    hello_world_node = Node(
        package='hello_world',
        executable='hello_world',
        output='screen',
        name='hello_world',
        parameters=[]
    )
    
    aestaetic_node = Node(
        package='aesthetic_control',
        executable='aesthetic_control',
        output='screen',
        name='aesthetic_control',
        parameters=[]
    )
    
        # publisher for rviz transformations
    remote_rviz_node = Node(
        package='remote_rviz',
        executable='remote_rviz',
        name='remote_rviz'
    )
    
    #publish robot position from simulation
    ego_robot_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ego_robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', os.path.join(get_package_share_directory('vehicle_control'), 'launch', 'ego_racecar.xacro')])}],
        remappings=[('/robot_description', 'ego_robot_description')]
    )
    
    #transforms
    transform_root_baselink = Node(
            package='tf2_ros',
            output='screen',
            executable='static_transform_publisher',
            arguments = ['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'root', '--child-frame-id', 'map']
        )

    # finalize
    ld.add_action(vehicle_control_node)
    ld.add_action(joy_linux_node)
    ld.add_action(lidar_launchfile)
    ld.add_action(vesc_launchfile)
    ld.add_action(vesc_odometer)
    ld.add_action(vesc_ackermann)

    ld.add_action(transform_imu_baselink)
    
    ld.add_action(ego_robot_publisher)
    ld.add_action(transform_root_baselink)
    ld.add_action(remote_rviz_node)
    
    
    ld.add_action(nav_lifecycle_node)
    ld.add_action(map_server_node)
    ld.add_action(amcl_node)
    # ld.add_action(slam_toolbox_node)

    ld.add_action(emergency_braking_node)
    ld.add_action(camera_node)
    ld.add_action(aestaetic_node)
    ld.add_action(gap_follow) # TODO Decide when to use gap follow and when to use pure pursuit
    #ld.add_action(pure_pursuit_node)

    return ld