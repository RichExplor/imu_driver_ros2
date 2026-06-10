# imu_driver_ros2

ROS2 IMU 驱动节点，支持串口通信、二进制协议解析和姿态解算（互补滤波 / RK4 四阶龙格-库塔）。

## 功能特性

- **串口通信**：基于 Boost.Asio 封装，支持带超时的同步读取
- **二进制协议解析**：支持自定义 IMU 二进制协议（帧头 `0x59 0x53`），自动校验和
- **姿态解算**：可配置算法和轴数模式
  - 互补滤波（Complementary Filter）：计算量小，适合嵌入式场景
  - RK4 四阶龙格-库塔（RK4 Integration）：高精度积分，适合高动态场景
  - 6 轴模式（加速度计 + 陀螺仪）：yaw 会漂移
  - 9 轴模式（加速度计 + 陀螺仪 + 磁力计）：全姿态无漂移
- **多消息发布**：同时支持自定义 `ImuData` 消息和标准 `sensor_msgs/Imu` + `sensor_msgs/MagneticField`
- **ROS2 参数机制**：所有参数均可通过 YAML 文件或命令行动态配置

## 项目结构

```
imu_driver_ros2/
├── CMakeLists.txt                          # 构建配置
├── package.xml                             # 包描述与依赖
├── config/
│   └── imu_params.yaml                     # 默认参数配置
├── launch/
│   └── imu_ros_publisher.launch.py         # ROS2 Python launch 文件
├── include/
│   ├── imu_driver_node.h                   # IMU 驱动顶层节点
│   ├── imu_parser.h                        # 二进制协议解析器
│   ├── imu_publisher.h                     # 消息发布器
│   ├── serial_port.h                       # 串口通信封装
│   └── algorithm/
│       ├── attitude_estimator.h            # 姿态解算管理器
│       ├── complementary_filter.h          # 互补滤波算法
│       ├── quaternion.h                    # 四元数工具类
│       └── rk4_integration.h              # RK4 积分算法
├── src/
│   ├── imu_driver_node.cpp                 # 节点实现
│   ├── imu_parser.cpp                      # 解析器实现
│   ├── imu_publisher.cpp                   # 发布器实现
│   ├── imu_ros_publisher.cpp               # 主入口
│   ├── serial_port.cpp                     # 串口实现
│   └── algorithm/
│       ├── attitude_estimator.cpp          # 姿态解算管理器实现
│       ├── complementary_filter.cpp        # 互补滤波实现
│       └── rk4_integration.cpp            # RK4 积分实现
└── plans/
    └── ros2_migration_plan.md              # ROS1 → ROS2 迁移计划
```

## 依赖

| 依赖 | 说明 |
|------|------|
| `rclcpp` | ROS2 C++ 客户端库 |
| `sensor_msgs` | 标准 IMU / MagneticField 消息 |
| `geometry_msgs` | Vector3 / Quaternion 消息 |
| `imu_driver_interfaces` | 自定义 `ImuData` 消息包 |
| `Eigen3` | 线性代数库（姿态解算核心） |
| `Boost (system)` | Boost.Asio 串口通信 |

## 编译

```bash
# 进入工作空间根目录
cd <catkin_ws>

# 编译（rosdep 会自动安装依赖）
# rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select imu_driver_interfaces
colcon build --packages-select imu_driver_ros2

# 加载环境
source install/setup.bash
```

> **注意**：编译前需确保 `imu_driver_interfaces` 包已存在于工作空间中并预先编译。

## 参数说明

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `port` | string | `/dev/ttyACM0` | 串口设备路径 |
| `baud` | int | `115200` | 波特率 |
| `timeout_ms` | int | `100` | 串口读取超时（毫秒），0 表示阻塞 |
| `publish_custom` | bool | `false` | 是否发布自定义 `ImuData` 消息 |
| `publish_sensor_msgs` | bool | `true` | 是否发布 `sensor_msgs/Imu` 与 `MagneticField` |
| `frame_id` | string | `imu_link` | 坐标系 ID |
| `enable_attitude_estimation` | bool | `true` | 是否启用姿态解算 |
| `algorithm_type` | string | `complementary` | 算法类型：`complementary`（互补滤波）或 `rk4`（四阶龙格库塔） |
| `axis_mode` | int | `9` | 轴数模式：`6`（加速度计+陀螺仪）或 `9`（加速度计+陀螺仪+磁力计） |
| `alpha_acc` | double | `0.02` | 加速度计融合/修正系数（0~1），越大越信任加速度计 |
| `alpha_mag` | double | `0.01` | 磁力计融合/修正系数（0~1），越大越信任磁力计 |

## 使用方法

### 1. 使用 launch 文件启动（推荐）

```bash
ros2 launch imu_driver_ros2 imu_ros_publisher.launch.py
```

可通过 launch 参数覆盖默认值：

```bash
ros2 launch imu_driver_ros2 imu_ros_publisher.launch.py \
    port:=/dev/ttyUSB1 \
    baud:=921600 \
    algorithm_type:=rk4 \
    axis_mode:=9
```

### 2. 使用 ros2 run 启动

```bash
# 使用默认参数
ros2 run imu_driver_ros2 imu_ros_publisher

# 使用参数文件
ros2 run imu_driver_ros2 imu_ros_publisher --ros-args \
    --params-file $(ros2 pkg prefix imu_driver_ros2)/share/imu_driver_ros2/config/imu_params.yaml

# 命令行指定参数
ros2 run imu_driver_ros2 imu_ros_publisher --ros-args \
    -p port:=/dev/ttyACM0 \
    -p baud:=115200 \
    -p algorithm_type:=complementary \
    -p axis_mode:=9
```

## 话题

### 发布话题

| 话题名 | 消息类型 | 说明 |
|--------|----------|------|
| `/A0100E/imu/data_raw` | `imu_driver_interfaces/msg/ImuData` | 自定义 IMU 数据（需 `publish_custom:=true`） |
| `/A0100E/imu/data` | `sensor_msgs/msg/Imu` | 标准 IMU 消息（含姿态四元数、角速度、线加速度） |
| `/A0100E/imu/mag` | `sensor_msgs/msg/MagneticField` | 磁场消息 |

## 二进制协议

IMU 使用自定义二进制协议，帧格式如下：

- **帧头**：`0x59 0x53`（2 字节）
- **数据字段**：小端字节序，32-bit 有符号整数（温度为 16-bit）
- **校验和**：帧尾校验，确保数据完整性

支持的数据字段 ID：

| ID | 数据 | 缩放系数 |
|----|------|----------|
| 加速度计 | ax, ay, az | `× 1e-6 m/s²` |
| 陀螺仪 | wx, wy, wz | `× 1e-6 deg/s` |
| `0x30` | 磁场归一化 hx, hy, hz | `× 1e-6` |
| `0x31` | 磁场强度 mx, my, mz | `× 0.001 mGauss` |
| `0x41` | 四元数 q0, q1, q2, q3 | `× 1e-6` |
| `0x40` | 欧拉角 pitch, roll, yaw | `× 1e-6 deg` |
| `0x01` | 温度 temp | `× 0.01 °C` |

## 架构设计

```
┌─────────────────────────────────────────────────────────┐
│                    ImuDriverNode                         │
│                  (rclcpp::Node)                          │
│                                                          │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌─────────┐ │
│  │SerialPort│→ │ImuParser │→ │Attitude  │→ │Imu      │ │
│  │          │  │          │  │Estimator │  │Publisher│ │
│  │ 串口读取  │  │ 协议解析  │  │ 姿态解算  │  │ 消息发布 │ │
│  └──────────┘  └──────────┘  └──────────┘  └─────────┘ │
│       ↑              ↑              ↑             ↑      │
│   port/baud     帧头0x59,0x53  complementary/   话题发布  │
│   timeout_ms    校验和验证     rk4, 6/9轴       frame_id │
└─────────────────────────────────────────────────────────┘
```

- [`SerialPort`](include/serial_port.h)：基于 Boost.Asio 的串口封装，支持带超时的同步读取
- [`ImuParser`](include/imu_parser.h)：二进制协议解析器，自动帧同步和校验
- [`AttitudeEstimator`](include/algorithm/attitude_estimator.h)：姿态解算管理器，可配置算法和轴数
- [`ImuPublisher`](include/imu_publisher.h)：消息发布器，支持自定义和标准 ROS2 消息

## 许可证

[Apache-2.0](https://www.apache.org/licenses/LICENSE-2.0)

## 维护者

guofeng (<1637850405@qq.com>)
