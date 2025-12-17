from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def get_package_share_directory(package_name):
    """Get the package share directory using ament_index_python."""
    from ament_index_python.packages import get_package_share_directory
    return get_package_share_directory(package_name)


def generate_launch_description():
    # Declare launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='physics_world.sdf',
        description='Choose one of the world files from `/gazebo_examples/worlds`'
    )

    # Get the launch directory
    gazebo_launch_dir = os.path.join(get_package_share_directory('gazebo_ros'), 'launch')

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_launch_dir, 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': PathJoinSubstitution([FindPackageShare('digital_twin_examples'), 'worlds', LaunchConfiguration('world')]),
            'verbose': 'true',
        }.items()
    )

    # Robot description parameter
    robot_desc = ''
    try:
        with open(os.path.join(get_package_share_directory('digital_twin_examples'), 'models', 'humanoid', 'basic_humanoid.urdf'), 'r') as infp:
            robot_desc = infp.read()
    except FileNotFoundError:
        # Fallback to a simple robot description if file not found
        robot_desc = """<?xml version="1.0"?>
        <robot name="basic_humanoid">
            <link name="base_link">
                <visual>
                    <geometry>
                        <box size="0.3 0.2 0.1"/>
                    </geometry>
                    <material name="blue">
                        <color rgba="0 0 1 0.8"/>
                    </material>
                </visual>
                <collision>
                    <geometry>
                        <box size="0.3 0.2 0.1"/>
                    </geometry>
                </collision>
                <inertial>
                    <mass value="5.0"/>
                    <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
                </inertial>
            </link>
        </robot>"""

    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_desc,
            'publish_frequency': 50.0,
            'use_sim_time': True
        }],
        remappings=[
            ('/joint_states', 'basic_humanoid/joint_states')
        ]
    )

    # Spawn entity node to load the robot into Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'basic_humanoid',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.0',
            '-robot_namespace', 'basic_humanoid'
        ],
        output='screen'
    )

    # Joint state publisher (for testing purposes)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': True,
            'source_list': ['basic_humanoid/joint_states'],
            'rate': 50.0
        }]
    )

    return LaunchDescription([
        world_arg,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        joint_state_publisher,
    ])