# =========================================================
# 第一步：打地基和起名字
# =========================================================
# 规定图纸的最低版本，就像规定“必须用哪年的新工具才能看懂这份图纸”
cmake_minimum_required(VERSION 3.20)

# 给我们的工厂（项目）起个名字
project(thrust_controler_server)

# 打开编译器的小助手，方便以后你用写代码软件（比如 VSCode）时有代码提示
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 让电脑变成一个“严厉的老师”，如果你代码写得不规范，就给你发警告
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# =========================================================
# 第二步：去仓库搬“乐高套件箱” (寻找依赖库)
# =========================================================
# REQUIRED 的意思是“必须找到！找不到就立刻罢工报错！”
# ament_cmake、nav_msgs、rclcpp、std_msgs 都是 ROS 2 的基础箱子
find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rov_interfaces REQUIRED) # 我们自己定义的专属消息箱子

# 下面三个是做数学计算和矩阵的箱子 (Eigen 和 OSQP)
find_package(Eigen3 REQUIRED)

# ──────────────────────────────────────────
# 第三方依赖：osqp + osqp-eigen (Eigen wrapper)
# ──────────────────────────────────────────
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(OSQP_BUILD_DEMO_EXE OFF CACHE BOOL "" FORCE)
set(OSQP_BUILD_UNITTESTS OFF CACHE BOOL "" FORCE)

# qdldl 是 osqp 的子依赖，指向本地缓存避免从 GitHub 下载
set(FETCHCONTENT_SOURCE_DIR_QDLDL
  ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/qdldl
  CACHE PATH "Local QDLDL source"
)

add_subdirectory(
  ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/osqp
  ${CMAKE_CURRENT_BINARY_DIR}/third_party/osqp
  EXCLUDE_FROM_ALL
)

# add_subdirectory 不生成 osqp::osqp 别名，手动创建以满足 osqp-eigen 的依赖检查
if(TARGET osqp AND NOT TARGET osqp::osqp)
  add_library(osqp::osqp ALIAS osqp)
endif()

# 跳过 osqp-eigen 的 try_compile 版本检测（子进程看不到父进程 alias target）
set(OSQP_IS_V1 TRUE)
set(OSQP_IS_V1_FINAL TRUE)

add_subdirectory(
  ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/osqp-eigen
  ${CMAKE_CURRENT_BINARY_DIR}/third_party/osqp-eigen
  EXCLUDE_FROM_ALL
)

# 压制第三方库的上游警告，只影响其自身 target
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  foreach(tgt OSQPLIB osqp osqpstatic)
    if(TARGET ${tgt})
      target_compile_options(${tgt} PRIVATE
        -Wno-parentheses -Wno-pedantic -Wno-implicit-fallthrough)
    endif()
  endforeach()
  if(TARGET OsqpEigen)
    target_compile_options(OsqpEigen PRIVATE -Wno-extra)
  endif()
endif()

# 下面三个是 Ignition Fortress 仿真器的箱子
find_package(ignition-msgs8 REQUIRED)
find_package(ignition-math6 REQUIRED)
find_package(ignition-transport11 REQUIRED)


# =========================================================
# 第三步：开始生产自己做的零件和玩具 (声明目标)
# =========================================================
# 1. 做一个“通用电池”（动态库）
# add_library 意思是做成一个可以给别人用的模块，不自己运行
add_library(ThrustAllocationSolver SHARED
  src/ThrustAllocationSolver.cpp
)

# 2. 做三个可以跑的“玩具车”（可执行文件，里面有 main 函数的程序）
# add_executable 的意思是把 .cpp 文件变成一个可以双击或者用命令运行的程序
add_executable(thrust_controler_server src/thrust_controler_server.cpp)
add_executable(pubilsh_robot_state src/pubilsh_robot_state.cpp)



# =========================================================
# 第四步：给它们发“地图” (配置头文件目录)
# =========================================================
# 电脑很笨，即使有了 .cpp 文件，它也不知道里面的 #include "xxx.h" 去哪找
# target_include_directories 就是告诉电脑去哪里找那些说明书（头文件）
target_include_directories(ThrustAllocationSolver PRIVATE
  /usr/include/eigen3
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_include_directories(thrust_controler_server PRIVATE
  /usr/include/eigen3
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)


# =========================================================
# 第五步：把“乐高套件”插到我们的“玩具”上 (链接库)
# =========================================================
# 1. 给通用电池插上数学工具包（OSQP 和 Eigen）
target_link_libraries(ThrustAllocationSolver
  osqp::osqp
  OsqpEigen::OsqpEigen
)

# 2. 组装推力控制器（它需要我们的通用电池，还需要数学工具和 ROS 2 基础）
# target_link_libraries 专门用来接 C++ 原生的库
target_link_libraries(thrust_controler_server
  ThrustAllocationSolver
  osqp::osqp
  OsqpEigen::OsqpEigen
)
# ament_target_dependencies 是 ROS 2 特有的快捷指令，专门用来接 ROS 2 的库
ament_target_dependencies(thrust_controler_server 
  rclcpp 
  std_msgs 
  rov_interfaces
  nav_msgs
)

# 3. 组装两个雷达状态转发器 (它们需要 Ignition 的库 和 ROS 2 的库)
# 给 pubilsh_robot_state 插上装备
target_link_libraries(pubilsh_robot_state
  ignition-msgs8::ignition-msgs8
  ignition-math6::ignition-math6
  ignition-transport11::ignition-transport11
)
ament_target_dependencies(pubilsh_robot_state 
  rclcpp 
  nav_msgs
  std_msgs 
  rov_interfaces
)



# =========================================================
# 第六步：打包上架商品 (安装规则)
# =========================================================
# 程序做好了，但还丢在工厂的车间里。
# install 的作用是把做好的程序放到商店的货架上（放到 Linux 的系统识别目录里）
# 只有 install 了，你在终端输入 `ros2 run thrust_controler_server xxx` 时系统才能找到它！

# 上架可执行文件 (DESTINATION lib/${PROJECT_NAME} 是 ROS 2 规定的货架位置)
install(TARGETS
  thrust_controler_server
  pubilsh_robot_state
  
  DESTINATION lib/${PROJECT_NAME}
)

# 上架动态库 (DESTINATION lib 是一般库文件的货架)
install(TARGETS
  ThrustAllocationSolver
  DESTINATION lib
)

# 上架第三方动态库，使其运行时能被找到
# osqp 和 OsqpEigen 通过 add_subdirectory 编译，但 install 规则需手动指定路径
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/third_party/osqp/out/libosqp.so
  DESTINATION lib
)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/lib/libOsqpEigen.so.0.11.0
  DESTINATION lib
)
install(CODE "execute_process(
  COMMAND ${CMAKE_COMMAND} -E create_symlink
    libOsqpEigen.so.0.11.0 libOsqpEigen.so
  WORKING_DIRECTORY \"${CMAKE_INSTALL_PREFIX}/lib\"
)")


# =========================================================
# 第七步：交工前的体检 (测试和收尾)
# =========================================================
# 这一块是用来做代码测试的，目前默认就行
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 告诉 ROS 2 系统：“这个叫 thrust_controler_server 的大礼包我做完啦，请打包验收！”
ament_package()