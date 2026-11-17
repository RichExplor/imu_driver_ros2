cmake_minimum_required(VERSION 3.8)
project(imu_driver_ros2)

# 默认 C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ===== 依赖查找 =====
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(imu_driver_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

# ===== ImpVQF AHRS SDK（预编译静态库，x86_64 Linux） =====
set(IMPVQF_LIB "${CMAKE_CURRENT_SOURCE_DIR}/third_party/impvqf/lib/x86_64-linux-gnu/libimpvqf.a")
if(NOT EXISTS "${IMPVQF_LIB}")
  message(FATAL_ERROR "ImpVQF static library not found: ${IMPVQF_LIB}")
endif()

# ===== 包含目录 =====
include_directories(include ${EIGEN3_INCLUDE_DIR} third_party/impvqf/include)

# ===== 库：算法模块（纯 C++/Eigen + ImpVQF，无 ROS 依赖） =====
add_library(imu_algorithm STATIC
  src/algorithm/impvqf_estimator.cpp
)
target_include_directories(imu_algorithm PUBLIC include ${EIGEN3_INCLUDE_DIR} third_party/impvqf/include)
target_link_libraries(imu_algorithm PUBLIC Eigen3::Eigen "${IMPVQF_LIB}" m)

# ===== 库：核心模块（依赖 ROS2） =====
add_library(imu_driver_core STATIC
  src/imu_driver_node.cpp
  src/imu_parser.cpp
  src/imu_publisher.cpp
  src/serial_port.cpp
  src/register_parser.cpp
)
target_include_directories(imu_driver_core PUBLIC include ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(imu_driver_core
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  imu_driver_interfaces
)
target_link_libraries(imu_driver_core imu_algorithm ${Boost_LIBRARIES})

# ===== 可执行文件 =====
add_executable(imu_ros_publisher src/imu_ros_publisher.cpp)
target_link_libraries(imu_ros_publisher imu_driver_core imu_algorithm ${Boost_LIBRARIES})
ament_target_dependencies(imu_ros_publisher
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  imu_driver_interfaces
)

# ===== Install 规则 =====
install(TARGETS imu_ros_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY third_party/impvqf/include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# ===== 测试 =====
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
