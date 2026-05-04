from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

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
            output='screen'
        ),

        Node(
            package='tello_driver',
            executable='vision_node',
            name='vision_node',
            output='screen'
        ),

        Node(
            package='tello_driver',
            executable='telemetry_node',
            name='telemetry_node',
            output='screen'
        ),
    ])