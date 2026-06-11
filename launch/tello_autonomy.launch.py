from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    show_preview = LaunchConfiguration('show_preview')
    model_path = LaunchConfiguration('model_path')
    stream_enable_sdk_init = LaunchConfiguration('stream_enable_sdk_init')
    command_mux_enable_sdk_init = LaunchConfiguration('command_mux_enable_sdk_init')
    enable_stream_on = LaunchConfiguration('enable_stream_on')
    enable_landing_base_node = LaunchConfiguration('enable_landing_base_node')
    landing_base_publish_mask = LaunchConfiguration('landing_base_publish_mask')
    enable_mission_node = LaunchConfiguration('enable_mission_node')
    mission_id = LaunchConfiguration('mission_id')
    mission_dry_run = LaunchConfiguration('mission_dry_run')
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
            'stream_enable_sdk_init',
            default_value='true',
            description='Send SDK command initialization when stream_node starts.',
        ),
        DeclareLaunchArgument(
            'command_mux_enable_sdk_init',
            default_value='false',
            description='Send SDK command initialization when command_mux_node starts.',
        ),
        DeclareLaunchArgument(
            'enable_stream_on',
            default_value='true',
            description='Send streamon command when stream_node starts.',
        ),
        DeclareLaunchArgument(
            'enable_landing_base_node',
            default_value='true',
            description='Start landing_base_node for blue/yellow landing base detection.',
        ),
        DeclareLaunchArgument(
            'landing_base_publish_mask',
            default_value='false',
            description='Publish the landing base combined mask for calibration.',
        ),
        DeclareLaunchArgument(
            'enable_mission_node',
            default_value='false',
            description='Start mission_node. Disabled by default for safety.',
        ),
        DeclareLaunchArgument(
            'mission_id',
            default_value='phase1_demo',
            description='Mission id used by mission_node.',
        ),
        DeclareLaunchArgument(
            'mission_dry_run',
            default_value='true',
            description='Run mission_node without publishing real autonomy commands.',
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
                'enable_sdk_init': ParameterValue(
                    stream_enable_sdk_init,
                    value_type=bool,
                ),
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
            executable='landing_base_node',
            output='screen',
            condition=IfCondition(enable_landing_base_node),
            parameters=[{
                'show_preview': ParameterValue(show_preview, value_type=bool),
                'publish_mask': ParameterValue(
                    landing_base_publish_mask,
                    value_type=bool,
                ),
            }],
        ),

        Node(
            package='tello_driver',
            executable='mission_node',
            output='screen',
            condition=IfCondition(enable_mission_node),
            parameters=[{
                'mission_id': mission_id,
                'dry_run': ParameterValue(mission_dry_run, value_type=bool),
            }],
        ),

        Node(
            package='tello_driver',
            executable='command_mux_node',
            output='screen',
            parameters=[{
                'enable_sdk_init': ParameterValue(
                    command_mux_enable_sdk_init,
                    value_type=bool,
                ),
                'start_armed': ParameterValue(start_armed, value_type=bool),
                'max_xy_speed': ParameterValue(max_xy_speed, value_type=float),
                'max_z_speed': ParameterValue(max_z_speed, value_type=float),
                'max_yaw_speed': ParameterValue(max_yaw_speed, value_type=float),
            }],
        ),
    ])
