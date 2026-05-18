from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

print("---------------------robot_type = x3---------------------")

def generate_launch_description():
    ekf_config = '/home/natdanai/ROS_Source_Code/yahboomcar_ws/function_package_without_WiFi_module/yahboomcar_ws/src/yahboomcar_bringup/param/ekf_yahboom.yaml'

    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        parameters=[{
            'use_mag': False,
            'publish_tf': False,
            'world_frame': 'enu'
        }],
        remappings=[
            ('imu/data_raw', '/imu'),
            ('imu/data', '/imu/data')
        ]
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config]
    )

    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('yahboomcar_description'),
                'launch',
                'description_launch.py'
            )
        )
    )

    base_link_to_imu_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_base_imu',
        arguments=['-0.002999', '-0.0030001', '0.031701', '0', '0', '0', 'base_link', 'imu_frame']
    )

    return LaunchDescription([
        imu_filter_node,
        ekf_node,
        base_link_to_imu_tf_node,
        description_launch
    ])
