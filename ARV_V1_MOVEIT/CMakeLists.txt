cmake_minimum_required(VERSION 3.22)
project(ARV_V1_MOVEIT)

# ============= 1. 查找所有依赖包 =============
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(glfw3 REQUIRED)
find_package(serial_driver REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(arv_v1_interfaces REQUIRED)

# ========== MuJoCo手动配置 ==========
# 设置MuJoCo安装路径
set(MUJOCO_DIR "$ENV{MUJOCO_PATH}")

# 头文件目录
set(MUJOCO_INCLUDE_DIRS "${MUJOCO_DIR}/include")

# 库文件
set(MUJOCO_LIBRARIES "${MUJOCO_DIR}/lib/libmujoco.so")

# 打印找到的路径（调试用）
message(STATUS "MuJoCo include: ${MUJOCO_INCLUDE_DIRS}")
message(STATUS "MuJoCo library: ${MUJOCO_LIBRARIES}")

# 包含头文件目录
include_directories(include)

# ============= 2. 创建动力学工具类库（先创建，后续节点会链接它）=============
add_library(dynamics_computer 
  src/dynamics_computer.cpp
)

# 工具类的依赖
ament_target_dependencies(dynamics_computer
  kdl_parser
  orocos_kdl
  Eigen3
)

# 工具类的头文件路径
target_include_directories(dynamics_computer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${orocos_kdl_INCLUDE_DIRS}
  ${kdl_parser_INCLUDE_DIRS}
)

# ============= 2.5 创建卡尔曼滤波器库 =============
add_library(kalman_filter
  src/kalman_filter.cpp
)

# 卡尔曼滤波器的依赖
target_link_libraries(kalman_filter
  Eigen3::Eigen
)

# 卡尔曼滤波器的头文件路径
target_include_directories(kalman_filter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

# ============= 2.6 创建级联PID控制器库 =============
add_library(cascade_pid
  src/cascade_pid.cpp
)

# 级联PID的头文件路径
target_include_directories(cascade_pid PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)

# ============= 3. 创建可执行文件 =============

# -------------------------------------------
# 力矩控制器 Action Server 节点 (Stage 4)
add_executable(torque_controller_node 
  src/torque_controller_node.cpp
)

# 链接库（包含工具类和卡尔曼滤波器）
target_link_libraries(torque_controller_node
  dynamics_computer  # ← 链接动力学工具类
  kalman_filter      # ← 链接卡尔曼滤波器
  cascade_pid        # ← 新增: 链接级联PID
  ${rclcpp_LIBRARIES}
  ${rclcpp_action_LIBRARIES}
  ${urdf_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
  ${kdl_parser_LIBRARIES}
  Eigen3::Eigen
)

# 头文件路径
target_include_directories(torque_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>  # ← 添加 src 目录以找到 kalman_filter.hpp
  $<INSTALL_INTERFACE:include>
  ${rclcpp_INCLUDE_DIRS}
  ${rclcpp_action_INCLUDE_DIRS}
  ${ament_index_cpp_INCLUDE_DIRS}
  ${urdf_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${kdl_parser_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# ROS2 依赖
ament_target_dependencies(torque_controller_node
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs
  trajectory_msgs
  control_msgs
  urdf
  kdl_parser
  orocos_kdl
  Eigen3
  ament_index_cpp
)

# -------------------------------------------

# MuJoCo接口节点 (Stage 4 - MuJoCo仿真)
add_executable(mujoco_interface_node 
  src/mujoco_interface_node.cpp
)

# 链接库
target_link_libraries(mujoco_interface_node
  ${rclcpp_LIBRARIES}
  ${MUJOCO_LIBRARIES}  # ← 使用手动配置的MuJoCo库
  glfw  # GLFW窗口库
)

# 头文件路径
target_include_directories(mujoco_interface_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${rclcpp_INCLUDE_DIRS}
  ${ament_index_cpp_INCLUDE_DIRS}
  ${MUJOCO_INCLUDE_DIRS}  # ← 添加MuJoCo头文件
)

# 设置运行时库路径 
set_target_properties(mujoco_interface_node PROPERTIES
  INSTALL_RPATH "${MUJOCO_DIR}"
  BUILD_WITH_INSTALL_RPATH TRUE
)

# ROS2 依赖
ament_target_dependencies(mujoco_interface_node
  rclcpp
  sensor_msgs
  std_msgs
  ament_index_cpp
)

# -------------------------------------------
# 硬件接口节点 (Sim2Real)
add_executable(hardware_interface_node
  src/hardware_interface_node.cpp
  src/Crc.cpp
)

# 链接库
target_link_libraries(hardware_interface_node
  ${rclcpp_LIBRARIES}
  ${serial_driver_LIBRARIES}
)

# 头文件路径
target_include_directories(hardware_interface_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${rclcpp_INCLUDE_DIRS}
  ${serial_driver_INCLUDE_DIRS}
)

# ROS2 依赖
ament_target_dependencies(hardware_interface_node
  rclcpp
  sensor_msgs
  std_msgs
  serial_driver
)

# -------------------------------------------
# 轨迹管理节点 (Phase 1)
add_executable(trajectory_manager_node
  src/trajectory_manager_node.cpp
)

# 链接库
target_link_libraries(trajectory_manager_node
  ${rclcpp_LIBRARIES}
  ${rclcpp_action_LIBRARIES}
  yaml-cpp
)

# 头文件路径
target_include_directories(trajectory_manager_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${rclcpp_INCLUDE_DIRS}
  ${rclcpp_action_INCLUDE_DIRS}
)

# ROS2 依赖
ament_target_dependencies(trajectory_manager_node
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs
  trajectory_msgs
  control_msgs
  arv_v1_interfaces
)

# -------------------------------------------
# Mission Executor Node (Application Layer v2.0)
add_executable(mission_executor_node
  src/mission_executor_node.cpp
)

target_link_libraries(mission_executor_node
  ${rclcpp_LIBRARIES}
  yaml-cpp  # For reading trajectory info
)

ament_target_dependencies(mission_executor_node
  rclcpp
  std_msgs
  arv_v1_interfaces
)

# ============= 4. 安装 =============

# 安装可执行文件和库
install(TARGETS
  dynamics_computer       # ← 工具类库
  torque_controller_node
  mujoco_interface_node
  hardware_interface_node
  trajectory_manager_node  # ← 轨迹管理节点
  mission_executor_node    # ← Application Layer Node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 文件
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE
)

# 安装配置文件
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

# 安装 Python 脚本
install(PROGRAMS
  DESTINATION lib/${PROJECT_NAME}
)

# ============= 5. ament_package() 必须在最后 =============
ament_package()