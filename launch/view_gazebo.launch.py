import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    package_name = 'tros_sldworks_pkg'

    pkg_share = get_package_share_directory(package_name)
    world_file = PathJoinSubstitution([pkg_share, 'world', 'rover_world.sdf'])
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot_description.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)

    # Paths to allow Gazebo to find package meshes and custom models (like ArUco markers)
    pkg_parent_share = os.path.abspath(os.path.join(pkg_share, '..'))
    models_dir = os.path.join(pkg_share, 'models')
    
    existing_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    
    # Construct combined GZ_SIM_RESOURCE_PATH
    resource_paths = [models_dir, pkg_parent_share, pkg_share]
    if existing_gz_path:
        resource_paths.append(existing_gz_path)
    
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join(resource_paths)
    )

    robot_description = robot_description_config.toxml()

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
        launch_arguments={'gz_args': [PathJoinSubstitution(['-r ', world_file])]}.items()
    )

    # Spawns the rover into Gazebo DIRECTLY from the SDF file
    gz_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'rover',
            '-z', '0.4'
        ],
        output='screen'
    )

    # Bridge Gazebo topics to ROS 2 using config file
    bridge_params = os.path.join(pkg_share, 'config', 'gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )

    # Lidar static transform publisher
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