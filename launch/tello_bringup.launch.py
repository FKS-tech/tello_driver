from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    enable_stream_node = LaunchConfiguration('enable_stream_node')
    enable_vision_node = LaunchConfiguration('enable_vision_node')

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_stream_node',
            default_value='true',
            description='Start tello_driver stream_node.',
        ),
        DeclareLaunchArgument(
            'enable_vision_node',
            default_value='true',
            description='Start tello_driver vision_node.',
        ),

        Node(
            package='joy',
            executable='joy_node',
            name='joy_node_input',
            output='screen'
        ),

        Node(
            package='tello_driver',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),

        Node(
            package='tello_driver',
            executable='stream_node',
            name='stream_node',
            output='screen',
            condition=IfCondition(enable_stream_node),
        ),

        Node(
            package='tello_driver',
            executable='vision_node',
            name='vision_node',
            output='screen',
            condition=IfCondition(enable_vision_node),
        ),

        Node(
            package='tello_driver',
            executable='telemetry_node',
            name='telemetry_node',
            output='screen'
        ),
    ])
