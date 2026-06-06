import os
import random

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _merge_resource_paths(*path_groups):
    merged_paths = []

    for group in path_groups:
        if not group:
            continue

        entries = group.split(os.pathsep) if isinstance(group, str) else group
        for entry in entries:
            if entry and entry not in merged_paths:
                merged_paths.append(entry)

    return os.pathsep.join(merged_paths)


def _spawn_robot(package_name, entity_name, x_position, y_position, z_position, yaw):
    package_share = get_package_share_directory(package_name)
    urdf_path = os.path.join(package_share, 'urdf', 'so101.urdf')

    return Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-world', 'empty',
            '-file', urdf_path,
            '-name', entity_name,
            '-x', str(x_position),
            '-y', str(y_position),
            '-z', str(z_position),
            '-Y', str(yaw),
            # Set initial pose so camera sees the table: joint_2 (shoulder) and joint_3 (elbow)
            '-initial_joint_positions', 'joint_2 0.6 joint_3 0.8',
        ],
    )


def _spawn_robot_from_topic(entity_name, topic_name, x_position, y_position, z_position, yaw):
    return Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-world', 'empty',
            '-topic', topic_name,
            '-name', entity_name,
            '-x', str(x_position),
            '-y', str(y_position),
            '-z', str(z_position),
            '-Y', str(yaw),
            # Set initial pose so camera sees the table: joint_2 (shoulder) and joint_3 (elbow)
            '-initial_joint_positions', 'joint_2 0.6 joint_3 0.8',
        ],
    )


def _robot_nodes(package_name, namespace, frame_prefix, condition=None):
    """Return [joint_state_publisher, robot_state_publisher] for one robot."""
    package_share = get_package_share_directory(package_name)
    urdf_path = os.path.join(package_share, 'urdf', 'so101.urdf')
    with open(urdf_path, 'r', encoding='utf-8') as f:
        urdf_content = f.read()

    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=namespace,
        arguments=[urdf_path],
        parameters=[{
            'use_robot_description_topic': False,
            'publish_default_positions': True,
            'use_sim_time': True,
        }],
        condition=condition,
        output='screen',
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        parameters=[{
            'robot_description': urdf_content,
            'use_sim_time': True,
        }],
        condition=condition,
        output='screen',
    )

    return [jsp, rsp]


def generate_launch_description():
    gazebo_share = get_package_share_directory('so101_gazebo')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    follower_moveit_share = get_package_share_directory('so101_moveit_config')
    follower_share = get_package_share_directory('so101_description')
    
    # Define pkg_description for the new nodes
    pkg_description = follower_share

    world_path = os.path.join(gazebo_share, 'worlds', 'empty_world.sdf')
    rviz_config = os.path.join(gazebo_share, 'config', 'follower_gazebo.rviz')
    
    # Path to controllers configuration
    robot_controllers = os.path.join(gazebo_share, 'config', 'controllers.yaml')

    resource_paths = _merge_resource_paths(
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
        [
            os.path.dirname(follower_share),
            gazebo_share,
        ],
    )

    ignition_resource_paths = _merge_resource_paths(
        os.environ.get('IGN_GAZEBO_RESOURCE_PATH', ''),
        [
            os.path.dirname(follower_share),
            gazebo_share,
        ],
    )

    # 1. Gazebo Simulation
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r -v 4 {world_path}',
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # 2. Robot Spawning Actions
    spawn_follower = _spawn_robot(
        package_name='so101_description',
        entity_name='so101_follower',
        x_position=-0.55,
        y_position=0.0,
        z_position=0.7774,
        yaw=0.0,
    )
    spawn_follower_from_topic = _spawn_robot_from_topic(
        entity_name='so101_follower',
        topic_name='robot_description',
        x_position=-0.55,
        y_position=0.0,
        z_position=0.7774,
        yaw=0.0,
    )

    # 3. Cup and Markings Spawning Actions
    spawn_cup = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', os.path.join(pkg_description, 'urdf', 'container.urdf'),
            '-name', 'target_container',
            '-x', '-0.23', 
            '-y', '0.0', 
            '-z', '0.8', 
        ],
        output='screen',
    )

    # NEW: Spawn the Blue Initial Marking at the same spot as the cup
    spawn_marking_initial = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', os.path.join(pkg_description, 'urdf', 'marking_initial.urdf'),
            '-name', 'marking_start',
            '-x', '-0.23', '-y', '0.0', '-z', '0.08', # Slightly lower to sit on table
        ],
        output='screen',
    )

    # NEW: Spawn the Red Final Marking at the destination
    spawn_marking_final = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', os.path.join(pkg_description, 'urdf', 'marking_final.urdf'),
            '-name', 'marking_goal',
            '-x', '-0.47', '-y', '-0.30', '-z', '0.78', 
        ],
        output='screen',
    )

    # 4. Controller Manager Node
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_controllers],
        output='screen',
        condition=LaunchConfigurationEquals('moveit', 'follower'),
    )

    # 5. RSP + JSP for visualization-only mode
    follower_nodes = _robot_nodes(
        'so101_description',
        'so101_follower',
        '',
        condition=LaunchConfigurationEquals('moveit', 'none'),
    )

    # Static TF for world to base_link
    tf_follower = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_world_follower',
        arguments=[
            '--x', '-0.55', '--y', '0.0', '--z', '0.7774',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'world',
            '--child-frame-id', 'base_link',
        ],
        condition=LaunchConfigurationEquals('moveit', 'none'),
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=LaunchConfigurationEquals('moveit', 'none'),
        output='screen',
    )

    # 6. MoveIt 2 Integration
    follower_rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(follower_moveit_share, 'launch', 'rsp.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items(),
        condition=LaunchConfigurationEquals('moveit', 'follower'),
    )

    follower_move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(follower_moveit_share, 'launch', 'move_group.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items(),
        condition=LaunchConfigurationEquals('moveit', 'follower'),
    )

    follower_moveit_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(follower_moveit_share, 'launch', 'moveit_rviz.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items(),
        condition=LaunchConfigurationEquals('moveit', 'follower'),
    )

    # 7. Controller Spawners
    spawn_controllers = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster', 
            'arm_controller', 
            'gripper_controller',
            '--param-file', robot_controllers
        ],
        condition=LaunchConfigurationEquals('moveit', 'follower'),
        output='screen',
    )

    # WATER PARTICLES LOGIC
    water_particle_nodes = []
    x_base = -0.23
    y_base = 0.0
    z_base = 0.10 

    # Use a much tighter spread (0.005 instead of 0.01)
    for i in range(20):
        water_particle_nodes.append(
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-file', os.path.join(pkg_description, 'urdf', 'water_particle.urdf'),
                    '-name', f'water_ball_{i}',
                    '-x', str(-0.23), # Exact center
                    '-y', str(0.0),   # Exact center
                    '-z', str(0.85 + (i * 0.005)) # Stack higher so they drop in
                ],
                output='screen',
            )
        )

    # Final Launch Description Construction
    ld_items = [
        DeclareLaunchArgument(
            'moveit',
            default_value='none',
            description='MoveIt integration mode: none or follower',
        ),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_paths),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', ignition_resource_paths),
        gazebo,
    ]
    
    # Common Visualization Nodes
    ld_items.extend([
        *follower_nodes,
        tf_follower,
        rviz,
    ])
    
    # Follower/MoveIt Specific Nodes
    ld_items.extend([
        follower_rsp,
        follower_move_group,
        follower_moveit_rviz,
        controller_manager,
    ])
    
    # Timed Sequence for Stability
    ld_items.extend([
        # Spawn robot first
        TimerAction(period=2.0, actions=[spawn_follower], condition=LaunchConfigurationEquals('moveit', 'none')),
        TimerAction(period=5.0, actions=[spawn_follower_from_topic], condition=LaunchConfigurationEquals('moveit', 'follower')),
        
        # Spawn controllers after robot is in Gazebo
        TimerAction(period=8.0, actions=[spawn_controllers], condition=LaunchConfigurationEquals('moveit', 'follower')),
        
        # Spawn the target container and markings last
        TimerAction(period=12.0, actions=[
            spawn_cup, 
            spawn_marking_initial, 
            spawn_marking_final
        ]),

        # Spawn water particles after container is ready
        TimerAction(period=15.0, actions=water_particle_nodes),
    ])
    
    return LaunchDescription(ld_items)