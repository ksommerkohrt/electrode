from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.actions import DeclareLaunchArgument, Shutdown, LogInfo, IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals, IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


ARGUMENTS = [

    # launch arguments
    DeclareLaunchArgument('use_sim_time',
        default_value=['false'],
        description='use simulation time'
    ),

    DeclareLaunchArgument('capabilities',
        default_value='[clientPublish,services,connectionGraph,assets]',
        description='capabilities for foxglove'
    ),

    DeclareLaunchArgument('gui',
        default_value='off',
        choices=['rviz', 'foxglove', 'off'],
        description='use rviz, foxglove, or gui off'
    ),

    DeclareLaunchArgument('joy',
        default_value='true',
        choices=['true', 'false'],
        description='use joystick'
    ),

    DeclareLaunchArgument('log_level',
        default_value=['warn'],
        description='Logging level'
    ),

    DeclareLaunchArgument('vehicle',
        default_value=['b3rb'],
        description='vehicle'
    ),
]

def generate_launch_description():

    joy = Node(
        package='joy',
        output='log',
        executable='joy_node',
        condition=IfCondition(LaunchConfiguration('joy')),
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        on_exit=Shutdown()
    )

    joy_throttle = Node(
        package='topic_tools',
        executable='throttle',
        condition=IfCondition(LaunchConfiguration('joy')),
        arguments=['messages', '/joy', '10', '/cerebri/in/joy'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        condition=LaunchConfigurationEquals('gui', 'rviz'),
        arguments=[
            '-d', [PathJoinSubstitution([FindPackageShare('electrode'), 'config',
            LaunchConfiguration('vehicle')]), '.rviz'], '--ros-args', '--log-level',
            LaunchConfiguration('log_level')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        on_exit=Shutdown(),
    )

    foxglove_websockets = IncludeLaunchDescription(
        XMLLaunchDescriptionSource([PathJoinSubstitution(
            [get_package_share_directory('foxglove_bridge'), 'launch', 'foxglove_bridge_launch.xml'])]),
        condition=LaunchConfigurationEquals('gui', 'foxglove'),
        launch_arguments=[('capabilities', LaunchConfiguration('capabilities')),
                        ('use_sim_time', LaunchConfiguration('use_sim_time'))])


    return LaunchDescription(ARGUMENTS + [
        joy,
        joy_throttle,
        rviz_node,
        foxglove_websockets
    ])
