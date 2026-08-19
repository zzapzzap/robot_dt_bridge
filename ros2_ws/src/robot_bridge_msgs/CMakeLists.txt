cmake_minimum_required(VERSION 3.8)
project(robot_bridge_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotMemory.msg"
  "msg/RobotCommand.msg"
  "msg/RobotControlState.msg"
  "msg/RobotPose.msg"
  "msg/RobotStatus.msg"
  "msg/SafetyMode.msg"
  "srv/GetRobotStatus.srv"
  "srv/SetSafetyMode.srv"
  "srv/GetSafetyMode.srv"
  "srv/RequestStart.srv"
  "srv/RequestStop.srv"
  "srv/SetSpeedPercent.srv"
  "srv/SetHold.srv"
  "srv/TriggerRobotAction.srv"
  DEPENDENCIES std_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
