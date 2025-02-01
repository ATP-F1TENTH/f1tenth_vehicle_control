
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

    vesc_parameters = { 'speed_to_erpm_gain' :             5254.0, #factor to compute forward motion
                        'speed_to_erpm_offset':            0.0,
                        'steering_angle_to_servo_gain':    -0.752,   #to convert steering angle to servo position
                        'steering_angle_to_servo_offset':  0.525   #makes the car drive straight
    }

    ld = LaunchDescription()  

    #autonomous driving controller
    gap_follower_node = Node(
        package='gap_follow',
        executable='gap_follower',
        output='screen',
        name='gap_follower',
        parameters=[],
        remappings=[('/drive', '/to_drive')] #gap following publishes to vehicle_control_node
    )

    pure_pursuit_node = Node(
        package='raceline',
        executable='pure_pursuit',
        output='screen',
        name='pure_pursuit_node'
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
                     'race_line_csv': race_track_path + '/my_map_raceline.csv',
                     'drive_topic': r'/to_drive',
                     'odom_topic':r'odom',
                     'odom_frame': r'ego_racecar/odom'
                     }]
    )

    # mpcc
    mpcc_config = os.path.join(
        get_package_share_directory('mpcc'),
        'config',
        'real_launch.yaml'
    )
    mpcc_node = Node(
        package='mpcc',
        executable='mpcc',
        output='screen',
        name='mpcc',
        parameters=[mpcc_config],
        remappings=[]
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

    #driver for the gamepad
    joy_linux_node = Node(
        package="joy_linux",
        executable="joy_linux_node",
        name="joy_linux_node",
        emulate_tty="true"
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

    vesc_odometer = IncludeLaunchDescription(
                        AnyLaunchDescriptionSource([
                            FindPackageShare("vesc_ackermann"), '/launch', '/vesc_to_odom_node.launch.xml']),
                        launch_arguments={
                            'speed_to_erpm_gain' :              str(vesc_parameters['speed_to_erpm_gain']),
                            'speed_to_erpm_offset':             str(vesc_parameters['speed_to_erpm_offset']),
                            'steering_angle_to_servo_gain':     str(vesc_parameters['steering_angle_to_servo_gain']),
                            'steering_angle_to_servo_offset':   str(vesc_parameters['steering_angle_to_servo_offset']),
                            'use_imu_to_calc_angular_velocity': str(True)
                        }.items()
                        )

    #ackermann driver for vesc
    #but instead of listening to /ackermann_cmd, we want to listen to /drive for input messages
    vesc_ackermann = Node(
        package="vesc_ackermann",
        executable="ackermann_to_vesc_node",
        name="ackermann_to_vesc_node",
        parameters=[{'speed_to_erpm_gain' :             vesc_parameters['speed_to_erpm_gain']},
                    {'speed_to_erpm_offset':            vesc_parameters['speed_to_erpm_offset']},
                    {'steering_angle_to_servo_gain':    vesc_parameters['steering_angle_to_servo_gain']},
                    {'steering_angle_to_servo_offset':  vesc_parameters['steering_angle_to_servo_offset']},
                   ],
        remappings=[('/ackermann_cmd', '/drive')]
    )


    #transforms
    transform_laser_imu = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '0.33', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'base_link', '--child-frame-id', 'laser']
        )
    transform_imu_baselink = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '0.23', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'base_link', '--child-frame-id', 'imu']
        )

    #robot model
    ego_robot_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ego_robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', os.path.join(get_package_share_directory('vehicle_control'), 'launch', 'ego_racecar.xacro')])}],
        remappings=[('/robot_description', 'ego_robot_description')]
    )

    aestaetic_node = Node(
        package='aesthetic_control',
        executable='aesthetic_control',
        output='screen',
        name='aesthetic_control',
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

    # finalize
    ld.add_action(vehicle_control_node)

    ld.add_action(pure_pursuit_node)
    # ld.add_action(paw_pure_pursuit_node)
    # ld.add_action(gap_follower_node)
    # ld.add_action(mpcc_node)

    ld.add_action(joy_linux_node)
    ld.add_action(lidar_launchfile)
    ld.add_action(vesc_launchfile)
    ld.add_action(vesc_odometer)
    ld.add_action(vesc_ackermann)
    ld.add_action(aestaetic_node)
    ld.add_action(camera_node)
    ld.add_action(ego_robot_publisher)

    ld.add_action(transform_laser_imu)
    ld.add_action(transform_imu_baselink)

    return ld