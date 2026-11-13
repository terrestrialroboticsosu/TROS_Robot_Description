"""Nav2 for the rover, stage 1 (odometry + GPS only).

Start the simulation first (view_gazebo.launch.py); it provides the EKFs,
navsat_transform and the drive controller this depends on. Then:

    ros2 launch tros_sldworks_pkg nav2.launch.py
    ros2 run tros_sldworks_pkg gps_mission
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('tros_sldworks_pkg')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    params = [params_file, {'use_sim_time': use_sim_time}]

    # controller_server and behavior_server both write to cmd_vel_nav; the
    # velocity smoother reads that and is the only node driving the wheels.
    to_smoother = [('cmd_vel', 'cmd_vel_nav')]
    to_wheels = to_smoother + [('cmd_vel_smoothed', '/diff_cont/cmd_vel')]

    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'velocity_smoother',
        'bt_navigator',
        'waypoint_follower',
    ]

    def nav2_node(package, executable, remappings=None):
        return Node(
            package=package,
            executable=executable,
            name=executable,
            output='screen',
            parameters=params,
            remappings=remappings or [],
        )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml')),

        nav2_node('nav2_controller', 'controller_server', to_smoother),
        nav2_node('nav2_planner', 'planner_server'),
        nav2_node('nav2_behaviors', 'behavior_server', to_smoother),
        nav2_node('nav2_velocity_smoother', 'velocity_smoother', to_wheels),
        nav2_node('nav2_bt_navigator', 'bt_navigator'),
        nav2_node('nav2_waypoint_follower', 'waypoint_follower'),

        # Brings the nodes above up in order and shuts them down together.
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': lifecycle_nodes,
            }],
        ),
    ])
