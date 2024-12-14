import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():

    # Set the package name and custom world file path
    package_name = 'my_bot'  # <--- CHANGE ME
    #world_file_path = os.path.join(
    #    get_package_share_directory(package_name),
    #    'worlds',
    #    'obstacles.world'  # Ensure this is the correct relative path
    #)

    # Include the robot_state_publisher launch file
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include the Gazebo launch file with a custom world file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        #launch_arguments={'world': world_file_path}.items()
    )

    # Run the spawner node from the gazebo_ros package
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'my_bot',
            '-x', '14.0',  # Set x-coordinate
            '-y', '0.0',  # Set y-coordinate
            '-z', '0.0',  # Set z-coordinate
            '-Y', '1.57'  # Set yaw (rotation around z-axis in radians)
        ],
        output='screen'
    )

    environment_visualizer = Node(
        package=package_name,  # Replace with the actual package name
        executable='environment_visualizer',      # Replace with the actual executable name
        name='environment_visualizer',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Add the trajectory_visualizer node
    trajectory_visualizer = Node(
        package=package_name,  # Replace with the actual package name
        executable='trajectory_visualizer',      # Replace with the actual executable name
        name='trajectory_visualizer',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Launch them all
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        environment_visualizer,
        trajectory_visualizer
    ])
