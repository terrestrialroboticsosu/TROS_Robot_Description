import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('tros_sldworks_pkg')
    world_file = PathJoinSubstitution([pkg_share, 'world', 'rover_world.sdf'])

    # Append package share directory to GZ_SIM_RESOURCE_PATH so 
    # model:// and package:// mesh URIs resolve automatically
    pkg_parent_share = os.path.abspath(os.path.join(pkg_share, '..'))
    existing_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f"{pkg_parent_share}:{existing_gz_path}" if existing_gz_path else pkg_parent_share
    )

    # 1. URDF File (Kept for ROS robot_state_publisher / RViz TF tree)
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'robot_description.urdf'])
    robot_description = Command(['cat ', urdf_file])

    # 2. SDF File (Used strictly for Gazebo)
    sdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'robot_description.sdf'])

    # Publishes robot TF and robot_description topic (requires URDF)
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen'
    )

    # Launches Gazebo Sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={'gz_args': PathJoinSubstitution(['-r ',world_file])}.items()
    )

    # Spawns the rover into Gazebo DIRECTLY from the SDF file
    gz_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', sdf_file,
            '-name', 'rover',
            '-z', '0.5'
        ],
        output='screen'
    )

    # Bridge Gazebo /velodyne_points topic to ROS 2
    bridge_params = os.path.join(get_package_share_directory('tros_sldworks_pkg'),'config','gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',]
    )

    # Add this alongside your other nodes
    lidar_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_static_tf',
        # Arguments: x y z yaw pitch roll parent_frame child_frame
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'rover/base_link/velodyne_vlp16']
    )
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        gz_resource_path,
        rsp,
        gz_sim,
        gz_spawn,
        ros_gz_bridge,
        lidar_static_tf,
        rviz
    ])
