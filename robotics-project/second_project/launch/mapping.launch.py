

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('second_project')

    # ── 1. PointCloud2 → LaserScan ────────────────────────────────────────────
    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/ugv/rslidar_points'),
            ('scan',     '/scan'),
        ],
        parameters=[{
            'target_frame':  'UGV_base_link', #rslidar --- UGV_base_link
            'transform_tolerance': 0.01, # 0.01 --- 0.5
            'min_height':    0.45, # -1.0 --- -0.4
            'max_height':    0.8, # 1.0 --- 1.5 0.55
            'angle_min':    -3.14159, 
            'angle_max':     3.14159,
            'angle_increment': 0.00436,
            'scan_time':     0.1,
            'range_min':     0.1,
            'range_max':     42.0, #30 --- 50
            'use_inf':       True,
            'use_sim_time':  True,
        }],
    )

    # ── 2. SLAM Toolbox  ────────────────
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(pkg, 'config', 'slam_toolbox_params.yaml'),
            {'use_sim_time': True},
        ],
    )

    # ── 3. RViz2 ─────────────────────────────────────────────────────────────
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg, 'rviz', 'mapping.rviz')],
        parameters=[{'use_sim_time': True}],
    )
 
    

    return LaunchDescription([
        pointcloud_to_laserscan,
        slam_toolbox,
        rviz2,
    ])
