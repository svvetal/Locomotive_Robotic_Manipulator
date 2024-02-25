import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ros_robot_description_dir = get_package_share_directory('ros_robot_description')
    ros_robot_description_share = os.path.join(get_package_prefix('ros_robot_description'), 'share')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')

    world_file_arg = DeclareLaunchArgument(
        name='world',
        default_value=os.path.join(
            get_package_share_directory('ros_robot_description'),
            'world',
            'my_world'
        ),
        description='Path to the Gazebo world file'
    )

    # Declare a launch argument to control whether to use sim time
    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Whether to use simulated time or not'
    )

    # Set Gazebo model path environment variable
    env_var = SetEnvironmentVariable('GAZEBO_MODEL_PATH', ros_robot_description_share)

    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=os.path.join(ros_robot_description_dir, 'urdf', 'ros_robot.xacro'),
        description='Absolute path to robot urdf file'
    )

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]), value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
        # Setting 'use_sim_time' to True for robot_state_publisher node
    )

    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gzclient.launch.py')
        )
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'ros_robot', '-topic', 'robot_description'],
        output='screen',
        parameters=[{'use_sim_time': True}]  
        # Setting 'use_sim_time' to True for spawn_robot node
    )

    return LaunchDescription([
        world_file_arg,
        use_sim_time_arg,  # Add the use_sim_time launch argument
        env_var,
        model_arg,
        start_gazebo_server,
        start_gazebo_client,
        robot_state_publisher_node,
        spawn_robot
    ])
