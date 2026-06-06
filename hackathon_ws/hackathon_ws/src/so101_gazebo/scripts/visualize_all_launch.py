import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Path Setup
    desc_pkg_share = get_package_share_directory('so101_description')
    
    urdf_files = {
        'robot': os.path.join(desc_pkg_share, 'urdf', 'so101.urdf'),
        'container': os.path.join(desc_pkg_share, 'urdf', 'container.urdf'),
        'marking_init': os.path.join(desc_pkg_share, 'urdf', 'marking_initial.urdf'),
        'marking_final': os.path.join(desc_pkg_share, 'urdf', 'marking_final.urdf')
    }

    # Helper function to read URDF content
    def get_urdf_content(path):
        with open(path, 'r') as f:
            return f.read()

    # 2. Coordinate Math (Based on your Gazebo Spawn positions)
    # Robot is at X: -0.55, Y: 0.0
    # Cup/Init is at X: -0.23, Y: 0.0 -> Relative Offset: +0.32, 0.0
    # Goal is at X: -0.51, Y: -0.185  -> Relative Offset: +0.04, -0.185

    return LaunchDescription([
        # --- STATE PUBLISHERS ---
        
        # Main Robot (with sliders)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': get_urdf_content(urdf_files['robot'])}]
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),

        # Static Container
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='rsp_container',
            parameters=[{'robot_description': get_urdf_content(urdf_files['container'])}],
            remappings=[('/robot_description', '/container_description')]
        ),

        # Initial Marking
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='rsp_m_init',
            parameters=[{'robot_description': get_urdf_content(urdf_files['marking_init'])}],
            remappings=[('/robot_description', '/marking_init_description')]
        ),

        # Final Marking
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='rsp_m_final',
            parameters=[{'robot_description': get_urdf_content(urdf_files['marking_final'])}],
            remappings=[('/robot_description', '/marking_final_description')]
        ),

        # --- STATIC TRANSFORMS (Connects "world" to model links) ---
        # Arguments: [x, y, z, yaw, pitch, roll, parent, child]

        # Robot Base Link
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link']),
        
        # Container & Initial Marking (+0.32m from robot)
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0.32', '0.0', '0.02', '0', '0', '0', 'world', 'container_link']),
        
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0.32', '0.0', '0.01', '0', '0', '0', 'world', 'marking_init_link']),
             
        # Final Marking (+0.04m in X, -0.185m in Y from robot)
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0.04', '-0.185', '0.01', '0', '0', '0', 'world', 'marking_final_link']),

        # --- RViz ---
        Node(package='rviz2', executable='rviz2', name='rviz2')
    ])