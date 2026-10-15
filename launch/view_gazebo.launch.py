import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    package_name = 'tros_sldworks_pkg'

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    pkg_share = get_package_share_directory(package_name)
    world_file = PathJoinSubstitution([pkg_share, 'world', 'rover_world.sdf'])
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot_description.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)

    # Resource paths for Gazebo models and package meshes
    pkg_parent_share = os.path.abspath(os.path.join(pkg_share, '..'))
    models_dir = os.path.join(pkg_share, 'models')
    
    existing_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    resource_paths = [models_dir, pkg_parent_share, pkg_share]
    if existing_gz_path:
        resource_paths.append(existing_gz_path)
    
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join(resource_paths)
    )

    robot_description = robot_description_config.toxml()

    # Robot State Publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Gazebo Sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={'gz_args': ['-r ', world_file]}.items()
    )

    # Spawn Robot Entity
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

    # ROS-Gazebo Topic Bridge
    bridge_params = os.path.join(pkg_share, 'config', 'gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Static TF Publisher for Lidar
    lidar_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_static_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'rover/base_link/velodyne_vlp16'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # ROS 2 Control Spawners
    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    diff_cont_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    arm_cont_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_cont'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 1. Local EKF Node -> Publishes /odometry/local and odom -> base_link TF
    local_ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'ekf.yaml'), {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', '/odometry/local')]
    )

    navsat_transform_node = Node(
            package='robot_localization', 
            executable='navsat_transform_node', 
            name='navsat_transform',
	        output='screen',
            parameters=[os.path.join(pkg_share, 'config', 'ekf.yaml'), {'use_sim_time': use_sim_time}],
            remappings=[('imu', 'imu/data'),
                        ('gps/fix', 'gps/fix'), 
                        ('gps/filtered', 'gps/filtered'),
                        ('odometry/gps', 'odometry/gps'),
                        ('odometry/filtered', 'odometry/global')]           

           ) 

    # 3. Global EKF Node -> Publishes /odometry/global and map -> odom TF
    global_ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'ekf.yaml'), {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', '/odometry/global')]
    )

    return LaunchDescription([
        use_sim_time_arg,
        gz_resource_path,
        rsp,
        gz_sim,
        gz_spawn,
        ros_gz_bridge,
        lidar_static_tf,
        rviz,
        local_ekf_node,
        navsat_transform_node,
        global_ekf_node,
        joint_broad_spawner,
        diff_cont_spawner,
        arm_cont_spawner
    ])