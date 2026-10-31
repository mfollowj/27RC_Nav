cmake_minimum_required(VERSION 3.10)
project(ros2_livox_simulation)

# ============================================================================
# Livox Mid-360 Gazebo 仿真插件（本工作区自带一份，用于修正上游插件的两个 bug）
#
# 为什么本仓库要自带一份而不是直接用 SCURC_Nav_Sim 里的那份：
#   src/livox_points_plugin.cpp 里改了两处（详见该文件的注释）：
#     1) 无回波射线不再发布 (0,0,0) 假点（上游每帧 5000 个点里 77% 是这种假点）；
#     2) 量程加回射线起点的 minDist，修掉"所有点距离系统性偏小 0.10 m"的非刚性畸变。
#   本工作区的 setup.bash 最后 source，本包会覆盖 SCURC_Nav_Sim 里的同名包。
#
# 上游 CMakeLists 里 target_link_libraries 写死了 libprotobuf.so.9 /
# libboost_chrono.so.1.71.0，本机没有这两个版本（系统是 .so.23 / .so.1.74.0），
# 因此这里改成按包/目标名链接。
# ============================================================================

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS chrono)
ament_auto_find_build_dependencies()

include_directories(
    include
    ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

ament_auto_add_library(ros2_livox SHARED
    src/livox_points_plugin.cpp
    src/livox_ode_multiray_shape.cpp
)

target_link_libraries(ros2_livox
    ${GAZEBO_LIBRARIES}
    RayPlugin
    GpuRayPlugin
    Boost::chrono
)

# scan_mode/mid360.csv 是指向 SCURC_Nav_Sim 源码目录的软链接（20MB，不重复拷贝）
ament_auto_package(
  INSTALL_TO_SHARE
  scan_mode
  urdf
)
