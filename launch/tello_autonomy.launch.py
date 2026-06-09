from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    show_preview = LaunchConfiguration('show_preview')
    model_path = LaunchConfiguration('model_path')
    enable_sdk_init = LaunchConfiguration('enable_sdk_init')
    enable_stream_on = LaunchConfiguration('enable_stream_on')
    start_armed = LaunchConfiguration('start_armed')
    max_xy_speed = LaunchConfiguration('max_xy_speed')
    max_z_speed = LaunchConfiguration('max_z_speed')
    max_yaw_speed = LaunchConfiguration('max_yaw_speed')

    return LaunchDescription([
        DeclareLaunchArgument(
            'show_preview',
            default_value='false',
            description='Enable OpenCV preview windows for vision nodes.',
        ),
        DeclareLaunchArgument(
            'model_path',
            default_value='yolov8n.pt',
            description='YOLO model used by vision_node.',
        ),
        DeclareLaunchArgument(
            'enable_sdk_init',
            default_value='true',
            description='Send SDK command initialization when stream_node and command_mux_node start.',
        ),
        DeclareLaunchArgument(
            'enable_stream_on',
            default_value='true',
            description='Send streamon command when stream_node starts.',
        ),
        DeclareLaunchArgument(
            'start_armed',
            default_value='false',
            description='Start command_mux_node accepting autonomous cmd_vel.',
        ),
        DeclareLaunchArgument(
            'max_xy_speed',
            default_value='30',
            description='Maximum autonomous RC speed for forward/back and left/right.',
        ),
        DeclareLaunchArgument(
            'max_z_speed',
            default_value='25',
            description='Maximum autonomous RC speed for up/down.',
        ),
        DeclareLaunchArgument(
            'max_yaw_speed',
            default_value='30',
            description='Maximum autonomous RC yaw speed.',
        ),

        Node(
            package='tello_driver',
            executable='stream_node',
            output='screen',
            parameters=[{
                'show_preview': ParameterValue(show_preview, value_type=bool),
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
            executable='vision_node',
            output='screen',
            parameters=[{
                'show_preview': ParameterValue(show_preview, value_type=bool),
                'model_path': model_path,
            }],
        ),

        Node(
            package='tello_driver',
            executable='qr_node',
            output='screen',
            parameters=[{
                'show_preview': ParameterValue(show_preview, value_type=bool),
            }],
        ),

        Node(
            package='tello_driver',
            executable='command_mux_node',
            output='screen',
            parameters=[{
                'enable_sdk_init': ParameterValue(enable_sdk_init, value_type=bool),
                'start_armed': ParameterValue(start_armed, value_type=bool),
                'max_xy_speed': ParameterValue(max_xy_speed, value_type=float),
                'max_z_speed': ParameterValue(max_z_speed, value_type=float),
                'max_yaw_speed': ParameterValue(max_yaw_speed, value_type=float),
            }],
        ),
    ])
