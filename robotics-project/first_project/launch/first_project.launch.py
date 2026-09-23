import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    pkg_share = get_package_share_directory('first_project')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'Rviz_Config.rviz')
    return LaunchDescription([
        
        Node(
            package='first_project',
            executable='odometer',
            name='odometer',
            parameters=[{'use_sim_time': True}],
        ),

        Node(
            package='first_project',
            executable='tf_error',
            name='tf_error',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),
    ])
