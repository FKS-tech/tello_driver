from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    show_preview = LaunchConfiguration('show_preview')
    model_path = LaunchConfiguration('model_path')
    stream_url = LaunchConfiguration('stream_url')

    return LaunchDescription([
        DeclareLaunchArgument(
            'show_preview',
            default_value='true',
            description='Enable OpenCV preview windows for stream and vision nodes.',
        ),
        DeclareLaunchArgument(
            'model_path',
            default_value='yolov8n.pt',
            description='YOLO model used by vision_node.',
        ),
        DeclareLaunchArgument(
            'stream_url',
            default_value='udp://0.0.0.0:11111?fifo_size=50000000&overrun_nonfatal=1',
            description='Tello UDP video stream URL.',
        ),

        Node(
            package='tello_driver',
            executable='stream_node',
            output='screen',
            parameters=[{
                'show_preview': ParameterValue(show_preview, value_type=bool),
                'stream_url': stream_url,
            }],
        ),

        Node(
            package='tello_driver',
            executable='telemetry_node',
            output='screen',
        ),

        Node(
            package='tello_driver',
            executable='vision_node',
            output='screen',
            parameters=[{
                'show_preview': ParameterValue(show_preview, value_type=bool),
                'model_path': model_path,
            }],
        ),
    ])
