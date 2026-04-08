cmake_minimum_required(VERSION 3.8)
project(robot_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)


# Ignition 依赖 (Fortress 版本)
find_package(ignition-cmake2 REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-msgs8 REQUIRED)
find_package(ignition-transport11 REQUIRED)

# ---------------------------------------------------------
# 2. 编译 Gazebo 插件 (生成 libTorquePlugin.so)
# ---------------------------------------------------------
add_library(TorquePlugin SHARED
  src/torque_plugin.cpp
)


target_link_libraries(TorquePlugin
  PRIVATE
    ignition-gazebo6::core
    ignition-msgs8::ignition-msgs8
    ignition-transport11::ignition-transport11
)


# ---------------------------------------------------------
# 3. 安装文件 (至关重要！)
# 只有放在这里，colcon build 才会把 .so 和可执行文件放进 install 文件夹
# ---------------------------------------------------------
install(TARGETS
  TorquePlugin      # 安装 Gazebo 插件库
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
