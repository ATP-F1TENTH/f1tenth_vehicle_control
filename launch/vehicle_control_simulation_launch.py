
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    ld = LaunchDescription()   

    #simulation
    sim_config = os.path.join(
        get_package_share_directory('f1tenth_gym_ros'),
        'config',
        'sim.yaml'
        )
    sim_config_dict = yaml.safe_load(open(sim_config, 'r'))
    bridge_node = Node(
        package='f1tenth_gym_ros',
        executable='gym_bridge',
        name='bridge',
        parameters=[sim_config]
    )

    #visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(get_package_share_directory('f1tenth_gym_ros'), 'launch', 'gym_bridge.rviz')]
    )

    #visualize the map in rviz
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        parameters=[{'yaml_filename': sim_config_dict['bridge']['ros__parameters']['map_path'] + '.yaml'},
                    {'topic': 'map'},
                    {'frame_id': 'map'},
                    {'output': 'screen'},
                    {'use_sim_time': True}]
    )
    #manage the map server
    nav_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    )

    #publish robot position from simulation
    ego_robot_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ego_robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', os.path.join(get_package_share_directory('f1tenth_gym_ros'), 'launch', 'ego_racecar.xacro')])}],
        remappings=[('/robot_description', 'ego_robot_description')]
    )

    #autonomous driving controller
    hello_world_node = Node(
        package='hello_world',
        executable='hello_world',
        output='screen',
        name='hello_world',
        parameters=[]
    )

    gap_follow_node = Node(
        package='gap_follow',
        executable='gap_follow',
        output='screen',
        name='gap_follow',
        parameters=[],
        remappings=[
            ('/to_drive', '/drive') # order: (from where, where to)
        ]
    )

    # pure pursuit race track follower
    race_track_path = os.path.join(
        get_package_share_directory('pure_pursuit'), 'config')
    
    paw_pure_pursuit_node = Node(
        package="pure_pursuit",
        executable="pure_pursuit",
        name="pure_pursuit",
        output="screen",
        parameters=[{
                     'race_line_csv': race_track_path + '/raceline_v1.csv',
                     'drive_topic': r'/drive',   # r'/to_drive'
                     'odom_topic':r'odom',
                     'odom_frame': r'ego_racecar/odom'
                     }]
    )

    pure_pursuit_node = Node(
        package='raceline',
        executable='pure_pursuit',
        output='screen',
        name='pure_pursuit_node',
        parameters=[],
        remappings=[
            ('/to_drive', '/drive') # order: (from where, where to)
        ]
    )

    mpcc_node = Node(
        package='mpcc',
        executable='mpcc',
        output='screen',
        name='mpcc',
        parameters=[],
        remappings=[
            ('/to_drive', '/drive') # order: (from where, where to)
        ]
    )

    #vehicle control - manual intervention for autonomous driving
    joy_config = os.path.join(
        get_package_share_directory('vehicle_control'),
        'config',
        'gamepad.yaml'
        )
    joy_config_dict = yaml.safe_load(open(joy_config, 'r'))
    vehicle_control_node = Node(
        package='vehicle_control',
        executable='vehicle_control',
        output='screen',
        name='vehicle_control',
        parameters=[joy_config_dict]
    )

    #driver for the gamepad - NOT USED
    joy_linux_node = Node(
        package="joy_linux",
        executable="joy_linux_node",
        emulate_tty="true"
    )


    #transforms required for mapping with nav2
    transform_odom_map = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'map', '--child-frame-id', 'ego_racecar/odom']
        )
    transform_odom_baselink = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'ego_racecar/odom', '--child-frame-id', 'ego_racecar/base_link']
        )


    # finalize
    ld.add_action(transform_odom_map)
    ld.add_action(transform_odom_baselink)
    ld.add_action(rviz_node)
    ld.add_action(bridge_node)
    ld.add_action(nav_lifecycle_node)
    ld.add_action(map_server_node)
    ld.add_action(ego_robot_publisher)
    ld.add_action(vehicle_control_node)

    # ld.add_action(hello_world_node)
    # ld.add_action(gap_follow_node)
    ld.add_action(pure_pursuit_node)
    # ld.add_action(paw_pure_pursuit_node)
    # ld.add_action(mpcc_node)

    return ld