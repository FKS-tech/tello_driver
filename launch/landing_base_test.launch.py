from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    show_preview = LaunchConfiguration('show_preview')
    stream_url = LaunchConfiguration('stream_url')
    enable_sdk_init = LaunchConfiguration('enable_sdk_init')
    enable_stream_on = LaunchConfiguration('enable_stream_on')
    publish_mask = LaunchConfiguration('publish_mask')

    return LaunchDescription([
        DeclareLaunchArgument(
            'show_preview',
            default_value='false',
            description='Enable OpenCV preview windows for stream and landing base nodes.',
        ),
        DeclareLaunchArgument(
            'stream_url',
            default_value='udp://0.0.0.0:11111?fifo_size=50000000&overrun_nonfatal=1',
            description='Tello UDP video stream URL.',
        ),
        DeclareLaunchArgument(
            'enable_sdk_init',
            default_value='true',
            description='Send SDK command initialization when stream_node starts.',
        ),
        DeclareLaunchArgument(
            'enable_stream_on',
            default_value='true',
            description='Send streamon command when stream_node starts.',
        ),
        DeclareLaunchArgument(
            'publish_mask',
            default_value='false',
            description='Publish the combined blue/yellow mask for calibration.',
        ),

        Node(
            package='tello_driver',
            executable='stream_node',
            output='screen',
            parameters=[{
                'show_preview': ParameterValue(show_preview, value_type=bool),
                'stream_url': stream_url,
                'enable_sdk_init': ParameterValue(enable_sdk_init, value_type=bool),
                'enable_stream_on': ParameterValue(enable_stream_on, value_type=bool),
            }],
        ),

        Node(
            package='tello_driver',
            executable='telemetry_node',
            output='screen',
        ),

        Node(
            package='tello_driver',
            executable='landing_base_node',
            output='screen',
            parameters=[{
                'show_preview': ParameterValue(show_preview, value_type=bool),
                'publish_mask': ParameterValue(publish_mask, value_type=bool),
            }],
        ),
    ])
