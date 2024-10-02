cmake_minimum_required(VERSION 3.8)
project(nuscenes2bag)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(message_interfaces REQUIRED)
find_package(Boost 1.71.0 REQUIRED COMPONENTS atomic filesystem iostreams locale program_options system thread)

add_executable(
  nuscenes_converter
  src/main.cpp
  src/EgoPoseConverter.cpp
  src/FileProgress.cpp
  src/ImageDirectoryConverter.cpp
  src/LidarDirectoryConverter.cpp
  src/LidarDirectoryConverterXYZIR.cpp
  src/MetaData.cpp
  src/MetaDataReader.cpp
  src/NuScenes2Bag.cpp
  src/RadarDirectoryConverter.cpp
  src/SceneConverter.cpp
  src/utils.cpp
  )
  
ament_target_dependencies(
  nuscenes_converter 
  rclcpp pcl_ros pcl_msgs pcl_conversions 
  std_msgs cv_bridge rosidl_default_generators 
  sensor_msgs geometry_msgs tf2 nav2_msgs 
  rosbag2_cpp Boost tf2_msgs
  message_interfaces)

install(TARGETS
  nuscenes_converter												
  DESTINATION lib/${PROJECT_NAME})

INCLUDE_DIRECTORIES(
  ${PROJECT_SOURCE_DIR}/include
  ${Boost_INCLUDE_DIRS}
  ${message_interfaces_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
)

target_link_libraries(
  nuscenes_converter
  ${message_interfaces_LIBRARIES}
  ${Boost_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  Boost::filesystem
  
)

ament_export_dependencies(Boost)
ament_export_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

set(CMAKE_PREFIX_PATH "/usr/lib/x86_64-linux-gnu")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_BUILD_TYPE Debug)

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
