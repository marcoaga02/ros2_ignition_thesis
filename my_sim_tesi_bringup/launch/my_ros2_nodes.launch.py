import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node

def generate_launch_description():
    pkg_project_bringup = get_package_share_directory('my_sim_tesi_bringup')
    coordinate_file = os.path.join(pkg_project_bringup, 'files', 'coordinates_goal.txt')

    pose_control_action_server = Node(
        package='my_sim_tesi_ros2_nodes',
        executable='pose_control_action_server',
        name='pose_control_action_server',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    pose_control_action_client = Node(
        package='my_sim_tesi_ros2_nodes',
        executable='pose_control_action_client',
        name='pose_control_action_client',
        output='screen',
        parameters=[{
            'file_path': coordinate_file,
            'use_sim_time': True
        }]
    )

    orchestrator_node = Node(
        package='my_sim_tesi_ros2_nodes',
        executable='orchestrator_node',
        name='orchestrator_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        pose_control_action_server,
        pose_control_action_client,
        orchestrator_node,
    ])
