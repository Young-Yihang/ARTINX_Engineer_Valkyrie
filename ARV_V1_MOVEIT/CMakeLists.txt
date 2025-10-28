cmake_minimum_required(VERSION 3.22)
project(ARV_V1_MOVEIT)

# ============= 1. 查找所有依赖包 =============
find_package(ament_cmake REQUIRED)
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
)

# 工具类的头文件路径
target_include_directories(dynamics_computer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${orocos_kdl_INCLUDE_DIRS}
  ${kdl_parser_INCLUDE_DIRS}
)

# ============= 3. 创建可执行文件 =============

# 动力学求解器观察者节点 (Stage 3)
add_executable(dynamics_solver_node 
  src/dynamics_solver_node.cpp
)

# 链接库（合并到一起）
target_link_libraries(dynamics_solver_node
  dynamics_computer  # ← 链接工具类
  ${rclcpp_LIBRARIES}
  ${urdf_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
  ${kdl_parser_LIBRARIES}
  Eigen3::Eigen
)

# 头文件路径
target_include_directories(dynamics_solver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${rclcpp_INCLUDE_DIRS}
  ${urdf_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${kdl_parser_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# ROS2 依赖
ament_target_dependencies(dynamics_solver_node
  rclcpp
  sensor_msgs
  std_msgs
  urdf
  kdl_parser
  orocos_kdl
  Eigen3
)

# -------------------------------------------

# 力矩控制器 Action Server 节点 (Stage 4)
add_executable(torque_controller_node 
  src/torque_controller_node.cpp
)

# 链接库（包含工具类）
target_link_libraries(torque_controller_node
  dynamics_computer  # ← 链接工具类
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
  $<INSTALL_INTERFACE:include>
  ${rclcpp_INCLUDE_DIRS}
  ${rclcpp_action_INCLUDE_DIRS}
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
)

# ============= 4. 安装 =============

# 安装可执行文件和库
install(TARGETS 
  dynamics_computer       # ← 新增：安装工具类库
  dynamics_solver_node
  torque_controller_node
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

# ============= 5. ament_package() 必须在最后 =============
ament_package()