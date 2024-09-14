import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution

from launch_ros.actions import Node

def generate_launch_description():
    pkg_project_bringup = get_package_share_directory('my_sim_tesi_bringup')
    pkg_project_gazebo = get_package_share_directory('my_sim_tesi_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    sdf_robot_scan  =  os.path.join(pkg_project_gazebo, 'models', 'pioneer2dx', 'model.sdf')
    with open(sdf_robot_scan, 'r') as infp:
        robot_scan_desc = infp.read()

    sdf_quadcopter  =  os.path.join(pkg_project_gazebo, 'models', 'quadcopter', 'model.sdf')
    with open(sdf_quadcopter, 'r') as infp:
        quadcopter_desc = infp.read()

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'ros_gz_bridge_final_config.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
            'use_sim_time': True,
        }],
        output='screen'
    )

    robot_scan_static_frame_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_map',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'robot_scan/odom']
    )

    quadcopter_static_frame_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_map',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'quadcopter/odom']
    )

    robot_scan_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='robot_scan',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_scan_desc},
        ]
    )

    quadcopter_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='quadcopter',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': quadcopter_desc},
        ]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_project_bringup, 'config', 'final_simulation_config.rviz')],
        parameters=[{'use_sim_time': True}]
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_project_gazebo,
            'worlds',
            'my_world.sdf -r',
        ]),
        'use_sim_time' : 'true'
        }.items()
    )

    my_ros2_nodes_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_bringup, 'launch', 'my_ros2_nodes.launch.py')
        )
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_bringup, 'launch', 'nav2.launch.py')
        )
    )

    return LaunchDescription([
        bridge,
        gz_sim,
        robot_scan_static_frame_map,
        quadcopter_static_frame_map,
        robot_scan_state_publisher,
        quadcopter_state_publisher,
        rviz,
        TimerAction(
            period=12.0,
            actions=[
                nav2_launch,
            ]
        ),
        TimerAction(
            period=20.0,
            actions=[
                my_ros2_nodes_launch,
            ]
        ),
    ])
