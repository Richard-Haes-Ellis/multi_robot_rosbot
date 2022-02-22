cmake_minimum_required(VERSION 3.5)
project(multi_robot_rosbot)

message(STATUS "Project name is: ${PROJECTNAME}") 

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(map_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# OpenCV is required for merging without initial positions
find_package(OpenCV REQUIRED)
if("${OpenCV_VERSION}" VERSION_LESS "3.0")
message(FATAL_ERROR "This package needs OpenCV >= 3.0")
endif()
if("${OpenCV_VERSION}" VERSION_LESS "4.0")
message(WARNING "This package supports OpenCV 3, but some features may not be "
"available. Upgrade to OpenCV 4 to take advantage of all features.")
endif()

## Specify additional locations of header files
include_directories(
  # ${Boost_INCLUDE_DIRS}
  # ${OpenCV_INCLUDE_DIRS}
  include
)

add_executable(merge_map
  src/merge_map.cpp
)

set(DEPENDENCIES
  rclcpp
  geometry_msgs
  image_geometry
  map_msgs
  nav_msgs
  tf2_geometry_msgs
  OpenCV
)
ament_target_dependencies(merge_map 
  ${DEPENDENCIES}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS 
  merge_map 
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/merge_map/
  DESTINATION include/map_merge/
)

install(DIRECTORY 
  config 
  launch 
  models 
  worlds 
  urdf 
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install Python executables
install(PROGRAMS
  scripts/publisher_member_function.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_package()