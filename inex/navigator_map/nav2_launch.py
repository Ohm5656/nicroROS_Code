import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    # 1. ประกาศตัวแปรสำหรับการตั้งค่า (Launch Configurations)
    open_rviz_arg = LaunchConfiguration('open_rviz')
    
    # เตรียม Path
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    yahboom_bringup_dir = get_package_share_directory('yahboomcar_bringup')
    
    map_file = os.path.abspath('my_robot_map.yaml')
    params_file = os.path.abspath(os.path.join('prarams', 'rpp_nav_params.yaml'))
    rviz_config_path = os.path.abspath(os.path.join('rviz', 'nav2_config.rviz'))

    # --- ส่วนของการประกาศ Argument เพื่อให้เลือกเปิด/ปิดได้จาก Terminal ---
    declare_open_rviz = DeclareLaunchArgument(
        'open_rviz',
        default_value='true',
        description='Whether to start RViz'
    )

    # --- เปิด RViz2 (ใส่ Condition ไว้) ---
    start_rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': False}],
        output='screen',
        condition=IfCondition(open_rviz_arg) # จะทำงานเมื่อ open_rviz เป็น true เท่านั้น
    )
    # --- ขั้นตอนที่ 2: รอ 3 วินาที แล้วเปิด Bringup หุ่นยนต์ ---
    start_bringup = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(yahboom_bringup_dir, 'launch', 'yahboomcar_bringup_launch.py')
                )
            )
        ]
    )

    # --- รอ 7 วินาที แล้วรัน Nav2 ---
    # --- แก้ไขในส่วน actions ของ start_nav ---
    start_nav = TimerAction(
        period=5.0, 
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
                ),
                launch_arguments={
                    'map': map_file,
                    'params_file': params_file,
                    'use_sim_time': 'False'
                }.items()
            ),
  
    # เชื่อม base_link -> laser_frame (เซนเซอร์ต้องติดบนตัวหุ่น)
    Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser',
        arguments=['-0.0046412', '0' , '0.094079', '0', '0', '0', 'base_link', 'laser_frame']
    ),
        ]
    )

    return LaunchDescription([
        declare_open_rviz,  # ต้องใส่ตัวประกาศ Argument ลงไปในนี้ด้วย
        start_rviz,
        start_bringup,
        start_nav
    ])
