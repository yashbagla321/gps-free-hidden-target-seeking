cmake_minimum_required(VERSION 3.16)
project(gps_free_seeking_msgs)
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RelayPacket.msg"
  "msg/RelayDiagnostics.msg"
  DEPENDENCIES builtin_interfaces
)
ament_package()
