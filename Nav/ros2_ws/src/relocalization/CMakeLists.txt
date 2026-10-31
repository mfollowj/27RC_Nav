cmake_minimum_required(VERSION 3.5)
project(relocalization)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wextra -Wpedantic)
set(CMAKE_BUILD_TYPE Release)

set(USE_LIVOX 1)
if(USE_LIVOX)
  add_definitions(-DUSE_LIVOX)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(small_gicp REQUIRED)
find_package(teaserpp REQUIRED)
if(USE_LIVOX)
  find_package(livox_ros_driver2 REQUIRED)
endif()

include_directories(${OpenMP_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

# ---- 6-DOF 全局重定位：TEASER++ 全局搜索 -> GICP 精配准 ----
add_executable(teaser_gicp_node src/teaser_gicp_node.cpp)
ament_target_dependencies(teaser_gicp_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_ros
  Eigen3
)
target_link_libraries(teaser_gicp_node
  teaserpp::teaser_registration
  teaserpp::teaser_features
  teaserpp::teaser_io
  small_gicp::small_gicp
  OpenMP::OpenMP_CXX
  ${PCL_LIBRARIES}
)
if(USE_LIVOX)
  ament_target_dependencies(teaser_gicp_node livox_ros_driver2)
endif()

# ---- 兼容保留：把 /icp_result 直接当 map→odom 发（新链路改用 reloc_bridge） ----
add_executable(transform_publisher src/transform_publisher.cpp)
ament_target_dependencies(transform_publisher
  rclcpp std_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs pcl_ros Eigen3)

install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})
install(TARGETS teaser_gicp_node transform_publisher
  DESTINATION lib/${PROJECT_NAME})

ament_package()
