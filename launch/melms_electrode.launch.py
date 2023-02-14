from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, Shutdown
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = True
    logger = LaunchConfiguration("log_level")
    return LaunchDescription([
        DeclareLaunchArgument(
            "log_level",
            default_value=["info"],
            description="Logging level"),

        Node(
            package='joy',
            output='log',
            executable='joy_node',
            arguments=['--ros-args', '--log-level', logger],
            parameters=[{'use_sim_time': use_sim_time}],
            on_exit=Shutdown()),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=[
                '-d',
                get_package_share_directory('electrode') + '/config/melms.rviz',
                '--ros-args', '--log-level', logger],
            parameters=[{'use_sim_time': use_sim_time}],
            on_exit=Shutdown(),
        ),
    ])
