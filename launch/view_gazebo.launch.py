import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            SetEnvironmentVariable, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import xacro
import yaml

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

    # The terrain's lowest point is world z=0, so the ground under the start
    # point is well above zero. terrain.yaml is written by tools/make_terrain.py.
    spawn_z = 0.4
    terrain_info = os.path.join(models_dir, 'urc_terrain', 'terrain.yaml')
    if os.path.exists(terrain_info):
        with open(terrain_info) as f:
            spawn_z += yaml.safe_load(f)['origin_ground_z']

    # Spawn Robot Entity
    gz_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'rover',
            '-z', str(spawn_z)
        ],
        output='screen'
    )

    # Gazebo's default view starts near z=0, which is underneath the terrain
    # (from below you only see its edge walls). Once the GUI is up, move the
    # view to just behind and above the rover.
    view_request = ('pose: {position: {x: -9, y: 0, z: %.2f}, '
                    'orientation: {x: 0, y: 0.149, z: 0, w: 0.989}}' % (spawn_z + 3.6))
    move_view = TimerAction(period=10.0, actions=[ExecuteProcess(
        cmd=['gz', 'service', '-s', '/gui/move_to/pose',
             '--reqtype', 'gz.msgs.GUICamera', '--reptype', 'gz.msgs.Boolean',
             '--timeout', '20000', '--req', view_request],
        output='log')])

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

    # Gazebo publishes the true position with no covariance. This adds a
    # realistic slowly-drifting GPS error and the covariance a receiver would
    # report (/gps/fix_raw -> /gps/fix). The error model's parameters and
    # defaults are in tros_sldworks_pkg/gps_covariance.py.
    gps_covariance = Node(
        package=package_name,
        executable='gps_covariance',
        name='gps_covariance',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'horizontal_stddev': 1.2,
            'vertical_stddev': 2.5,
        }]
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
            name='navsat_transform_node',  # must match the key in ekf.yaml
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
        move_view,
        ros_gz_bridge,
        gps_covariance,
        rviz,
        local_ekf_node,
        navsat_transform_node,
        global_ekf_node,
        joint_broad_spawner,
        diff_cont_spawner,
        arm_cont_spawner
    ])