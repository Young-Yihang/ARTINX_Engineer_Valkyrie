cmake_minimum_required(VERSION 3.22)
project(ARV_V1_MOVEIT)

# ============= 1. 查找所有依赖包（必须在 ament_package() 之前）=============
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)


# ============= 2. 添加可执行文件（必须在 target_link_libraries 之前）=============
add_executable(dynamics_solver_node 
  src/dynamics_solver_node.cpp
)

# ============= 3. 链接依赖库 =============
target_link_libraries(dynamics_solver_node
  ${rclcpp_LIBRARIES}
  ${urdf_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
  ${kdl_parser_LIBRARIES}
  Eigen3::Eigen
)

# ============= 4. 添加包含目录 =============
target_include_directories(dynamics_solver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${rclcpp_INCLUDE_DIRS}
  ${urdf_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${kdl_parser_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# ============= 5. 告诉 ament 这个 target 需要哪些依赖 =============
ament_target_dependencies(dynamics_solver_node
  rclcpp
  sensor_msgs
  std_msgs
  urdf
  kdl_parser
  orocos_kdl
  Eigen3
)

# ============= 6. 安装 =============
install(TARGETS dynamics_solver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE
)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

# ============= 7. ament_package() 必须在最后 =============
ament_package()