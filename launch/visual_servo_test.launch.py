from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    show_preview = LaunchConfiguration('show_preview')
    model_path = LaunchConfiguration('model_path')
    stream_url = LaunchConfiguration('stream_url')
    enable_sdk_init = LaunchConfiguration('enable_sdk_init')
    enable_stream_on = LaunchConfiguration('enable_stream_on')
    target_class_name = LaunchConfiguration('target_class_name')
    confidence_threshold = LaunchConfiguration('confidence_threshold')
    min_area_ratio = LaunchConfiguration('min_area_ratio')
    max_area_ratio = LaunchConfiguration('max_area_ratio')
    yaw_kp = LaunchConfiguration('yaw_kp')
    max_yaw_speed = LaunchConfiguration('max_yaw_speed')
    center_deadband = LaunchConfiguration('center_deadband')
    target_selection_strategy = LaunchConfiguration('target_selection_strategy')
    enable_target_lock = LaunchConfiguration('enable_target_lock')
    target_lock_timeout = LaunchConfiguration('target_lock_timeout')
    target_lock_max_error_distance = LaunchConfiguration('target_lock_max_error_distance')

    return LaunchDescription([
        DeclareLaunchArgument(
            'show_preview',
            default_value='false',
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
        DeclareLaunchArgument(
            'enable_sdk_init',
            default_value='false',
            description='Send SDK command initialization when stream_node starts.',
        ),
        DeclareLaunchArgument(
            'enable_stream_on',
            default_value='false',
            description='Send streamon command when stream_node starts.',
        ),
        DeclareLaunchArgument(
            'target_class_name',
            default_value='person',
            description='Target class used by visual_servo_node.',
        ),
        DeclareLaunchArgument(
            'confidence_threshold',
            default_value='0.6',
            description='Minimum detection confidence used by visual_servo_node.',
        ),
        DeclareLaunchArgument(
            'min_area_ratio',
            default_value='0.02',
            description='Minimum target area ratio used by visual_servo_node.',
        ),
        DeclareLaunchArgument(
            'max_area_ratio',
            default_value='1.0',
            description='Maximum target area ratio used by visual_servo_node.',
        ),
        DeclareLaunchArgument(
            'yaw_kp',
            default_value='35.0',
            description='Yaw proportional gain used by visual_servo_node.',
        ),
        DeclareLaunchArgument(
            'max_yaw_speed',
            default_value='30.0',
            description='Maximum absolute yaw speed used by visual_servo_node.',
        ),
        DeclareLaunchArgument(
            'center_deadband',
            default_value='0.10',
            description='Normalized horizontal deadband used by visual_servo_node.',
        ),
        DeclareLaunchArgument(
            'target_selection_strategy',
            default_value='closest_to_center',
            description='Target selection strategy used by visual_servo_node.',
        ),
        DeclareLaunchArgument(
            'enable_target_lock',
            default_value='true',
            description='Keep tracking the nearest previous target across frames.',
        ),
        DeclareLaunchArgument(
            'target_lock_timeout',
            default_value='1.0',
            description='Maximum age in seconds for target lock reuse.',
        ),
        DeclareLaunchArgument(
            'target_lock_max_error_distance',
            default_value='0.35',
            description='Maximum normalized distance for target lock reuse.',
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
            executable='vision_node',
            output='screen',
            parameters=[{
                'show_preview': ParameterValue(show_preview, value_type=bool),
                'model_path': model_path,
            }],
        ),

        Node(
            package='tello_driver',
            executable='visual_servo_node',
            output='screen',
            parameters=[{
                'target_class_name': target_class_name,
                'confidence_threshold': ParameterValue(confidence_threshold, value_type=float),
                'min_area_ratio': ParameterValue(min_area_ratio, value_type=float),
                'max_area_ratio': ParameterValue(max_area_ratio, value_type=float),
                'yaw_kp': ParameterValue(yaw_kp, value_type=float),
                'max_yaw_speed': ParameterValue(max_yaw_speed, value_type=float),
                'center_deadband': ParameterValue(center_deadband, value_type=float),
                'target_selection_strategy': target_selection_strategy,
                'enable_target_lock': ParameterValue(enable_target_lock, value_type=bool),
                'target_lock_timeout': ParameterValue(target_lock_timeout, value_type=float),
                'target_lock_max_error_distance': ParameterValue(
                    target_lock_max_error_distance,
                    value_type=float,
                ),
            }],
        ),
    ])
