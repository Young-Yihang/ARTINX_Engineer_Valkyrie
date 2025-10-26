cmake_minimum_required(VERSION 3.8)
project(ARV_V1_MODEL)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)

# 安装目录
install(DIRECTORY
  launch
  meshes
  urdf
  config
  DESTINATION share/${PROJECT_NAME}/
)
#
ament_package()