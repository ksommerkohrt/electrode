from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # launch substiutions
    use_sim_time = LaunchConfiguration('use_sim_time')
    logger = LaunchConfiguration('log_level')
    vehicle = LaunchConfiguration('vehicle')

    # launch arguments
    arg_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value=['true'],
        description='use simulation time'
    )

    arg_log_level = DeclareLaunchArgument(
        'log_level',
        default_value=['warn'],
        description='Logging level'
    )

    arg_vehicle = DeclareLaunchArgument(
        'vehicle',
        default_value=['mrb3s'],
        description='vehicle'
    )

    joy_node = Node(
        package='joy',
        output='log',
        executable='joy_node',
        arguments=['--ros-args', '--log-level', logger],
        parameters=[{'use_sim_time': use_sim_time}],
        on_exit=Shutdown()
    )

    path = [PathJoinSubstitution([FindPackageShare('electrode'), 'config', vehicle]), '.rviz']

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d', path,
            '--ros-args', '--log-level', logger],
        parameters=[{'use_sim_time': use_sim_time}],
        on_exit=Shutdown(),
    )

    return LaunchDescription([
        arg_use_sim_time,
        arg_log_level,
        arg_vehicle,
        joy_node,
        rviz_node
    ])
