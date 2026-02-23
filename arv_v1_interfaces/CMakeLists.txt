cmake_minimum_required(VERSION 3.22)
project(arv_v1_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(service_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # trajectory_manager_node 服务
  "srv/trajectory_manager/ListTrajectories.srv"
  "srv/trajectory_manager/LoadTrajectory.srv"
  "srv/trajectory_manager/SaveTrajectory.srv"
  "srv/trajectory_manager/SaveLastTrajectory.srv"
  # mission_executor_node 服务
  "srv/mission_executor/ExecuteAction.srv"
  "srv/mission_executor/GetTaskState.srv"
  # cartesian_executor_node 服务
  "srv/cartesian_executor/MoveToCartesianRPY.srv"
  "srv/cartesian_executor/StopCartesianMotion.srv"
  # gripper 控制服务
  "srv/GripperControl.srv"
  DEPENDENCIES trajectory_msgs service_msgs geometry_msgs
)

ament_package()
