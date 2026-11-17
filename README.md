# imu_driver_ros2

ROS2 IMU 驱动节点，适配汉威 DA901S 高性能姿态测量单元，通过固定协议轮询 + 寄存器协议读取板载姿态/温度；内部姿态由 ImpVQF SDK（预编译静态库）解算。设备姿态与 SDK AHRS 姿态分别发布到两个话题。

## 功能特性

- **串口通信**：基于 Boost.Asio 封装，支持带超时的同步读取
- **DA901S 固定协议解析**：帧头 `0x4E 0x4A`，16 位累加校验和，主机查询帧 `FF AA 27 34 00`
- **寄存器协议读取**：读取 `0x46~0x55` 一次性获得设备板载欧拉角、四元数与温度
- **双姿态话题**：`/DA901S/imu/attitude` 为设备读取的姿态；`/DA901S/imu/attitude_ahrs` 为内部 SDK AHRS 姿态
- **内部 AHRS**：ImpVQF SDK（`third_party/impvqf`，x86_64 预编译静态库，ABI v2 / float），输出 quat9d；配置参数见下表
- **发布单位**：加速度 m/s²、角速度 **°/s**、磁场 **mGauss**、温度 °C（角速度/磁场非 ROS 标准 SI 单位）
- **多消息发布**：自定义 `ImuData`、标准 `sensor_msgs/Imu` + `MagneticField`、温度 `std_msgs/Float32`，以及两个 `ImuAttitude` 话题
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
│   ├── imu_parser.h                        # 固定协议解析器
│   ├── register_parser.h                   # 寄存器协议解析器
│   ├── imu_publisher.h                     # 消息发布器
│   ├── serial_port.h                       # 串口通信封装
│   └── algorithm/
│       └── impvqf_estimator.h              # ImpVQF SDK C++ 封装
├── src/
│   ├── imu_driver_node.cpp                 # 节点实现
│   ├── imu_parser.cpp                      # 固定协议解析实现
│   ├── register_parser.cpp                 # 寄存器协议解析实现
│   ├── imu_publisher.cpp                   # 发布器实现
│   ├── imu_ros_publisher.cpp               # 主入口
│   ├── serial_port.cpp                     # 串口实现
│   └── algorithm/
│       └── impvqf_estimator.cpp            # ImpVQF SDK 封装实现
├── third_party/
│   └── impvqf/                             # ImpVQF SDK：头文件 + x86_64 静态库
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
| `Eigen3` | 线性代数库 |
| `Boost (system)` | Boost.Asio 串口通信 |
| ImpVQF SDK | `third_party/impvqf`（`libimpvqf.a`，x86_64 Linux，链接 `-lm`） |

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
| `port` | string | `/dev/ttyUSB0` | 串口设备路径 |
| `baud` | int | `230400` | 波特率（DA901S 默认值） |
| `timeout_ms` | int | `20` | 串口读取超时（毫秒），0 表示阻塞 |
| `poll_rate_hz` | int | `200` | 主机轮询频率（设备默认应答模式，需主动查询） |
| `publish_custom` | bool | `false` | 是否发布自定义 `ImuData` 消息 |
| `publish_sensor_msgs` | bool | `true` | 是否发布 `sensor_msgs/Imu` 与 `MagneticField` |
| `publish_temperature` | bool | `true` | 是否发布 `/DA901S/imu/temperature`（°C） |
| `publish_attitude` | bool | `true` | 是否发布两个姿态话题（设备姿态 + SDK AHRS 姿态） |
| `euler_unit` | string | `deg` | 姿态话题中欧拉角单位：`deg`（度）或 `rad`（弧度） |
| `enable_device_attitude` | bool | `true` | 是否读取设备板载姿态/温度寄存器 `0x46~0x55` |
| `register_poll_divider` | int | `10` | 每 N 次传感器轮询读一次寄存器（200Hz/10=20Hz） |
| `frame_id` | string | `imu_link` | 坐标系 ID |
| `enable_attitude_estimation` | bool | `true` | 是否启用内部 SDK AHRS（ImpVQF） |
| `ahrs_tau_acc` | double | `3.0` | 倾角滤波时间常数 (s)，SDK 默认 3.0 |
| `ahrs_tau_mag` | double | `9.0` | 磁航向修正时间常数 (s)，SDK 默认 9.0 |
| `ahrs_options` | int | `15` | SDK 选项位：1运动零偏\|2静止零偏\|4磁扰拒绝\|8梯形输入 |

## 使用方法

### 1. 使用 launch 文件启动（推荐）

```bash
ros2 launch imu_driver_ros2 imu_ros_publisher.launch.py
```

可通过 launch 参数覆盖默认值：

```bash
ros2 launch imu_driver_ros2 imu_ros_publisher.launch.py \
    port:=/dev/ttyUSB0 \
    baud:=230400 \
    poll_rate_hz:=200 \
    ahrs_tau_acc:=3.0 \
    ahrs_tau_mag:=9.0
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
    -p port:=/dev/ttyUSB0 \
    -p baud:=230400 \
    -p poll_rate_hz:=200 \
    -p ahrs_options:=15
```

## 话题

### 发布话题

| 话题名 | 消息类型 | 说明 |
|--------|----------|------|
| `/DA901S/imu/data_raw` | `imu_driver_interfaces/msg/ImuData` | 自定义 IMU 数据（需 `publish_custom:=true`） |
| `/DA901S/imu/data` | `sensor_msgs/msg/Imu` | 标准 IMU 消息（姿态四元数来自**内部 SDK AHRS**；角速度 °/s） |
| `/DA901S/imu/mag` | `sensor_msgs/msg/MagneticField` | 磁场消息（单位 **mGauss**） |
| `/DA901S/imu/temperature` | `std_msgs/msg/Float32` | 设备板载温度（单位 **°C**） |
| `/DA901S/imu/attitude` | `imu_driver_interfaces/msg/ImuAttitude` | **设备读取姿态**：寄存器四元数/欧拉角 |
| `/DA901S/imu/attitude_ahrs` | `imu_driver_interfaces/msg/ImuAttitude` | **内部 SDK AHRS 姿态**：ImpVQF quat9d 输出 |

> 两个话题消息均为四元数 `orientation` + 欧拉角 `euler`（x=roll, y=pitch, z=yaw；单位由 `euler_unit` 决定，默认 **度**）。`euler` 由对应四元数按 ZYX（yaw-pitch-roll）顺序换算，保证与 `orientation` 一致；万向锁附近 roll/yaw 存在多解性。

## DA901S 固定协议

设备默认 LV TTL 接口，波特率 `230400`，工作于**应答模式**（RATE 寄存器默认值 `0x00`），需由主机主动查询。

**主机查询帧**（5 字节）：`FF AA 27 34 00`（读取全部传感器数据）

**设备响应帧**（24 字节，小端序）：

| 偏移 | 字段 | 说明 |
|------|------|------|
| 0~1 | `4E 4A` | 帧头 |
| 2 | `0x13` | 长度（1 字节命令 + 18 字节数据） |
| 3 | `0x01` | 命令码（传感器数据；`0x00` 为零偏数据） |
| 4~21 | Ax/Ay/Az, Wx/Wy/Wz, Hx/Hy/Hz | 9 个 int16 原始数据 |
| 22~23 | `SUM_L SUM_H` | 校验和：帧头到 HzH 全部字节算术和，取低 16 位，低字节在前 |

物理量换算（RR = 原始 int16）：

- 加速度：`RR / 32768 * 16 * 9.80665` → m/s²
- 角速度：`RR / 32768 * 2000` → **°/s**（发布值；传给 SDK 时内部转 rad/s）
- 磁场：`RR` → **mGauss**（1 LSB = 1 mGauss；SDK 输入同尺度即可）

> 注意：角速度 °/s 与磁场 mGauss 与 ROS 标准（rad/s、Tesla）不同，下游节点需按此单位解读。

### 寄存器协议（设备板载姿态/温度）

主机请求帧（7 字节）：`FF AA 27 REG NUM SUM_L SUM_H`，`SUM` 为各字节算术和取低 16 位、低字节在前。

本驱动一次读取 `0x46~0x55` 共 16 个寄存器：请求 `FF AA 27 46 10 26 02`。

响应帧：`4E 4A NUM REG data[16] SUM_L SUM_H`（`NUM = 0x11` = 1 + 16）。

| 数据偏移 | 寄存器 | 内容 | 换算 |
|----------|--------|------|------|
| 0~1 | `0x46~0x47` | Roll | `raw / 32768 * 180°` |
| 2~3 | `0x48~0x49` | Pitch | `raw / 32768 * 180°` |
| 4~5 | `0x4A~0x4B` | Yaw | `raw / 32768 * 180°` |
| 6~7 | `0x4C~0x4D` | Temperature | `raw / 100 °C` |
| 8~15 | `0x4E~0x55` | Q0~Q3 | `raw / 32768`（Q0 为标量 w） |

寄存器数据有效时，设备姿态发布到 `/DA901S/imu/attitude`；温度经 `/DA901S/imu/temperature` 发布。内部 SDK AHRS 姿态独立发布到 `/DA901S/imu/attitude_ahrs`。

## 架构设计

```
┌─────────────────────────────────────────────────────────┐
│                    ImuDriverNode                         │
│                  (rclcpp::Node)                          │
│                                                          │
│  ┌──────────┐  ┌──────────┐  ┌───────────┐  ┌─────────┐ │
│  │SerialPort│→ │ImuParser │→ │ImpVQF SDK │→ │Imu      │ │
│  │          │  │Register  │  │AHRS 解算   │  │Publisher│ │
│  │ 串口读取  │  │Parser    │  │(静态库)    │  │ 消息发布 │ │
│  └──────────┘  └──────────┘  └───────────┘  └─────────┘ │
│       ↑              ↑              ↑             ↑      │
│   port/baud     帧头0x4E,0x4A   tau_acc/tau_mag  设备姿态  │
│   timeout_ms    校验和验证     options/采样率   AHRS姿态  │
└─────────────────────────────────────────────────────────┘
```

- [`SerialPort`](include/serial_port.h)：基于 Boost.Asio 的串口封装，支持带超时的同步读取
- [`ImuParser`](include/imu_parser.h)：DA901S 固定协议解析器，自动帧同步与校验和验证
- [`RegisterParser`](include/register_parser.h)：寄存器协议解析器，读取设备板载姿态/温度
- [`ImpvqfEstimator`](include/algorithm/impvqf_estimator.h)：ImpVQF SDK C++ 封装，内部 AHRS 解算
- [`ImuPublisher`](include/imu_publisher.h)：消息发布器，发布设备姿态与 SDK AHRS 姿态两个话题

## 许可证

[Apache-2.0](https://www.apache.org/licenses/LICENSE-2.0)

## 维护者

guofeng (<1637850405@qq.com>)
