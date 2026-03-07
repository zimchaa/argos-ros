cmake_minimum_required(VERSION 3.8)
project(argos_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AhrsData.msg"
  "msg/JointSpeeds.msg"
  "msg/IrProximity.msg"
  "msg/FlotillaData.msg"
  DEPENDENCIES std_msgs
)

ament_package()
