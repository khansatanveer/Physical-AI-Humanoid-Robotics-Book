#!/usr/bin/env python3
"""
Complete Digital Twin Launch File

This launch file brings up the complete digital twin system including:
- Gazebo simulation with physics and sensors
- Robot state publisher
- Sensor processing nodes
- Gazebo-Unity bridge
- Visualization tools
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Robot state publisher node (loads the URDF)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description':
                '<?xml version="1.0"?>'
                '<robot name="basic_humanoid">'
                '  <link name="base_link">'
                '    <visual>'
                '      <geometry><box size="0.2 0.1 0.1"/></geometry>'
                '    </visual>'
                '  </link>'
                '  <link name="torso">'
                '    <visual>'
                '      <geometry><box size="0.15 0.15 0.3"/></geometry>'
                '    </visual>'
                '  </link>'
                '  <joint name="base_to_torso" type="fixed">'
                '    <parent link="base_link"/>'
                '    <child link="torso"/>'
                '    <origin xyz="0 0 0.2"/>'
                '  </joint>'
                '</robot>'
        }]
    )

    # Joint state publisher (for simulation)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
    )

    # Sensor processing node
    sensor_processor = Node(
        package='ros2_integration_examples',
        executable='sensor_processor_node.py',
        name='sensor_processor',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
    )

    # Sensor fusion node
    sensor_fusion = Node(
        package='ros2_integration_examples',
        executable='sensor_fusion_node.py',
        name='sensor_fusion',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
    )

    # Gazebo-Unity bridge node
    gazebo_unity_bridge = Node(
        package='ros2_integration_examples',
        executable='gazebo_unity_bridge.py',
        name='gazebo_unity_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'unity_ip': '127.0.0.1',
            'unity_port': 5555,
            'publish_rate': 30
        }]
    )

    # Launch Gazebo with a world file
    gazebo_launch = ExecuteProcess(
        cmd=[
            'gz', 'sim', '-r',  # Run in real-time mode
            PathJoinSubstitution([
                FindPackageShare('gazebo_examples'),
                'worlds',
                'physics_world.sdf'
            ])
        ],
        output='screen'
    )

    # Return the launch description
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),

        # Launch everything
        robot_state_publisher,
        joint_state_publisher,
        sensor_processor,
        sensor_fusion,
        gazebo_unity_bridge,
        gazebo_launch
    ])


if __name__ == '__main__':
    generate_launch_description()