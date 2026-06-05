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
        default_value='115200',
        description='Serial baud rate',
    )
    timeout_ms_arg = DeclareLaunchArgument(
        'timeout_ms',
        default_value='100',
        description='Serial read timeout in milliseconds, 0 for blocking',
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
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='imu_link',
        description='Frame ID for IMU messages',
    )
    enable_attitude_estimation_arg = DeclareLaunchArgument(
        'enable_attitude_estimation',
        default_value='true',
        description='Whether to enable attitude estimation',
    )
    algorithm_type_arg = DeclareLaunchArgument(
        'algorithm_type',
        default_value='complementary',
        description='Attitude estimation algorithm: complementary or rk4',
    )
    axis_mode_arg = DeclareLaunchArgument(
        'axis_mode',
        default_value='9',
        description='Axis mode: 6 (accel+gyro) or 9 (accel+gyro+mag)',
    )
    alpha_acc_arg = DeclareLaunchArgument(
        'alpha_acc',
        default_value='0.02',
        description='Accelerometer fusion coefficient (0~1)',
    )
    alpha_mag_arg = DeclareLaunchArgument(
        'alpha_mag',
        default_value='0.01',
        description='Magnetometer fusion coefficient (0~1)',
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
            'publish_custom': LaunchConfiguration('publish_custom'),
            'publish_sensor_msgs': LaunchConfiguration('publish_sensor_msgs'),
            'frame_id': LaunchConfiguration('frame_id'),
            'enable_attitude_estimation': LaunchConfiguration('enable_attitude_estimation'),
            'algorithm_type': LaunchConfiguration('algorithm_type'),
            'axis_mode': LaunchConfiguration('axis_mode'),
            'alpha_acc': LaunchConfiguration('alpha_acc'),
            'alpha_mag': LaunchConfiguration('alpha_mag'),
        }],
    )

    return LaunchDescription([
        # Launch 参数
        port_arg,
        baud_arg,
        timeout_ms_arg,
        publish_custom_arg,
        publish_sensor_msgs_arg,
        frame_id_arg,
        enable_attitude_estimation_arg,
        algorithm_type_arg,
        axis_mode_arg,
        alpha_acc_arg,
        alpha_mag_arg,
        # 节点
        imu_driver_node,
    ])
