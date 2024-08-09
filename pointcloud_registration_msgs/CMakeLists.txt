cmake_minimum_required(VERSION 3.5)
project(pointcloud_registration_msgs)

find_package(ament_cmake)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(srv_files
  "srv/Registration.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  DEPENDENCIES builtin_interfaces sensor_msgs geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime builtin_interfaces sensor_msgs geometry_msgs std_msgs)
ament_package()
