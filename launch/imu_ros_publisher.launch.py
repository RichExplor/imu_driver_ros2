"""ROS2 Python launch 文件：启动 IMU 驱动节点"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """生成 launch 描述，声明所有可配置参数并传递给节点"""

    # ===== 声明 launch 参数 =====
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='Serial port device path, e.g. /dev/ttyUSB0',
    )
    baud_arg = DeclareLaunchArgument(
        'baud',
        default_value='230400',
        description='Serial baud rate (DA901S default: 230400)',
    )
    timeout_ms_arg = DeclareLaunchArgument(
        'timeout_ms',
        default_value='20',
        description='Serial read timeout in milliseconds, 0 for blocking',
    )
    poll_rate_hz_arg = DeclareLaunchArgument(
        'poll_rate_hz',
        default_value='200',
        description='Host polling rate in Hz (DA901S defaults to request-response mode)',
    )
    publish_custom_arg = DeclareLaunchArgument(
        'publish_custom',
        default_value='false',
        description='Whether to publish custom ImuData message',
    )
    publish_sensor_msgs_arg = DeclareLaunchArgument(
        'publish_sensor_msgs',
        default_value='true',
        description='Whether to publish sensor_msgs/Imu and MagneticField',
    )
    publish_temperature_arg = DeclareLaunchArgument(
        'publish_temperature',
        default_value='true',
        description='Whether to publish device temperature on /DA901S/imu/temperature (deg C)',
    )
    publish_attitude_arg = DeclareLaunchArgument(
        'publish_attitude',
        default_value='true',
        description='Whether to publish device attitude and SDK AHRS attitude topics',
    )
    euler_unit_arg = DeclareLaunchArgument(
        'euler_unit',
        default_value='deg',
        description='Euler angle unit in attitude topics: deg (default) or rad',
    )
    enable_device_attitude_arg = DeclareLaunchArgument(
        'enable_device_attitude',
        default_value='true',
        description='Whether to read device attitude/temperature registers 0x46~0x55',
    )
    register_poll_divider_arg = DeclareLaunchArgument(
        'register_poll_divider',
        default_value='10',
        description='Read registers once every N sensor polls',
    )
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='imu_link',
        description='Frame ID for IMU messages',
    )
    enable_attitude_estimation_arg = DeclareLaunchArgument(
        'enable_attitude_estimation',
        default_value='true',
        description='Whether to enable internal ImpVQF SDK AHRS',
    )
    ahrs_tau_acc_arg = DeclareLaunchArgument(
        'ahrs_tau_acc',
        default_value='3.0',
        description='ImpVQF accelerometer tilt filter time constant in seconds (SDK default 3.0)',
    )
    ahrs_tau_mag_arg = DeclareLaunchArgument(
        'ahrs_tau_mag',
        default_value='9.0',
        description='ImpVQF magnetic heading correction time constant in seconds (SDK default 9.0)',
    )
    ahrs_options_arg = DeclareLaunchArgument(
        'ahrs_options',
        default_value='15',
        description='ImpVQF options bitmask: 1 motion bias | 2 rest bias | 4 mag rejection | 8 trapezoid',
    )

    # ===== IMU 驱动节点 =====
    imu_driver_node = Node(
        package='imu_driver_ros2',
        executable='imu_ros_publisher',
        name='imu_ros_publisher',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baud': LaunchConfiguration('baud'),
            'timeout_ms': LaunchConfiguration('timeout_ms'),
            'poll_rate_hz': LaunchConfiguration('poll_rate_hz'),
            'publish_custom': LaunchConfiguration('publish_custom'),
            'publish_sensor_msgs': LaunchConfiguration('publish_sensor_msgs'),
            'publish_temperature': LaunchConfiguration('publish_temperature'),
            'publish_attitude': LaunchConfiguration('publish_attitude'),
            'euler_unit': LaunchConfiguration('euler_unit'),
            'enable_device_attitude': LaunchConfiguration('enable_device_attitude'),
            'register_poll_divider': LaunchConfiguration('register_poll_divider'),
            'frame_id': LaunchConfiguration('frame_id'),
            'enable_attitude_estimation': LaunchConfiguration('enable_attitude_estimation'),
            'ahrs_tau_acc': LaunchConfiguration('ahrs_tau_acc'),
            'ahrs_tau_mag': LaunchConfiguration('ahrs_tau_mag'),
            'ahrs_options': LaunchConfiguration('ahrs_options'),
        }],
    )

    return LaunchDescription([
        # Launch 参数
        port_arg,
        baud_arg,
        timeout_ms_arg,
        poll_rate_hz_arg,
        publish_custom_arg,
        publish_sensor_msgs_arg,
        publish_temperature_arg,
        publish_attitude_arg,
        euler_unit_arg,
        enable_device_attitude_arg,
        register_poll_divider_arg,
        frame_id_arg,
        enable_attitude_estimation_arg,
        ahrs_tau_acc_arg,
        ahrs_tau_mag_arg,
        ahrs_options_arg,
        # 节点
        imu_driver_node,
    ])
