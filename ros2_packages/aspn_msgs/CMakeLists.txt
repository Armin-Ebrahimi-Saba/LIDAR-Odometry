cmake_minimum_required(VERSION 3.8)
project(aspn_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TypeHeader.msg"
  "msg/TypeTimestamp.msg"
  "msg/TypeIntegrity.msg"
  "msg/MeasurementIMU.msg"
  "msg/MeasurementAttitude3D.msg"
  DEPENDENCIES std_msgs
)

ament_package()
