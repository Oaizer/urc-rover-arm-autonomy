cmake_minimum_required(VERSION 3.8)
project(urc_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TagDetection.msg" "msg/TagDetections.msg"
  "srv/SolveIK.srv" "srv/ComputeFK.srv"
  "msg/MappedTag.msg" "msg/TagMap.msg" "msg/PanelObservation.msg"
  "action/MoveArm.action"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces action_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
