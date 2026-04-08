cmake_minimum_required(VERSION 3.8)
project(rov_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 将所有的 .msg, .srv 添加到这里
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetForce.srv"
)

ament_package()