cmake_minimum_required(VERSION 3.8)
project(message_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/RadarObject.msg"
    "msg/RadarObjects.msg" 
    DEPENDENCIES geometry_msgs std_msgs sensor_msgs
)

# rosidl_generate_interfaces(${PROJECT_NAME}
#     # "msg/RadarObject.msg"
#     "msg/RadarObjects.msg" 
#     DEPENDENCIES geometry_msgs std_msgs sensor_msgs
# )

# add_definitions(-D_WIN32_WINNT=0x0601)

# add_executable(
#   message_interfaces
# )
  
# ament_target_dependencies(
#   message_interfaces 
#   rclcpp 
#   std_msgs rosidl_default_generators 
#   sensor_msgs geometry_msgs 
#   rosbag2_cpp)

# install(TARGETS
#   message_interfaces												# 修改4
#   DESTINATION lib/${PROJECT_NAME})

# target_link_libraries(
#   message_interfaces
# )

set(CMAKE_PREFIX_PATH "/usr/lib/x86_64-linux-gnu")

ament_export_dependencies(rosidl_default_runtime)

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
