import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory('yahboomcar_nav')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_path = LaunchConfiguration('maps')
    nav2_param_path = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        DeclareLaunchArgument(
            'maps',
            default_value=os.path.join(package_path, 'maps', 'yahboom_map.yaml'),
            description='Full path to map file to load'
        ),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(package_path, 'params', 'dwb_nav_params.yaml'),
            description='Full path to param file to load'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path
            }.items(),
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_base_laser',
            arguments=[
                '-0.0046412', '0', '0.094079',
                '0', '0', '0',
                'base_link', 'laser_frame'
            ],
            output='screen'
        ),

        Node(
            package='yahboomcar_nav',
            executable='stop_car',
            output='screen'
        )
    ])
