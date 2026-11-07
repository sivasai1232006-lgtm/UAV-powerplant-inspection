cmake_minimum_required(VERSION 3.8)
project(uav_interfaces)

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Task.msg"
  "msg/UAVState.msg"
  "msg/TaskStatus.msg"
  "msg/InspectionWaypoint.msg"
  "srv/ReassignTask.srv"
  "srv/GetUnassignedTasks.srv"
  "srv/AssignTask.srv"
  "action/GoToWaypoint.action"
  DEPENDENCIES action_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
