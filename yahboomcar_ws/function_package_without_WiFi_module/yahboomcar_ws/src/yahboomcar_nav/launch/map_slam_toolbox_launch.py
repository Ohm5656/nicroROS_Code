from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'odom_frame': 'odom',
                'map_frame': 'map',
                'base_frame': 'base_footprint',
                'scan_topic': '/scan',

                'resolution': 0.05,
                'map_update_interval': 1.0,
                'transform_publish_period': 0.05,

                'minimum_travel_distance': 0.05,
                'minimum_travel_heading': 0.08,

                'throttle_scans': 1,
                'max_laser_range': 3.0,

                'use_scan_matching': True,
                'use_scan_barycenter': True,
                'do_loop_closing': True,
                'loop_search_maximum_distance': 3.0,

                'scan_buffer_size': 5,
                'scan_buffer_maximum_scan_distance': 2.0,
                'correlation_search_space_dimension': 0.5,
                'correlation_search_space_resolution': 0.01,

                'debug_logging': False
            }]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_base_laser',
            arguments=['-0.0046412', '0', '0.094079', '0', '0', '0', 'base_link', 'laser_frame']
        ),
    ])
