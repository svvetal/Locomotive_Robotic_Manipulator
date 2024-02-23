import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    ros_robot_description_dir = get_package_share_directory('ros_robot_description')

    # Declare a launch argument to control whether to use sim time
    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Whether to use simulated time or not'
    )

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

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        parameters=[{'use_sim_time': True}]
        # Setting 'use_sim_time' to True for joint_state_publisher_gui_node
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(ros_robot_description_dir, 'rviz', 'display.rviz')],
        parameters=[{'use_sim_time': True}]
        # Setting 'use_sim_time' to True for rviz_node
    )

    return LaunchDescription([
        use_sim_time_arg,  # Add the use_sim_time launch argument
        model_arg,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])
