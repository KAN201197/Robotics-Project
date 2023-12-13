cmake_minimum_required(VERSION 3.8)
project(gazebo_simulation)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_plugins REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_ros_control REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/wheeltec_robot_gazebo.cpp
# )

# Add cmake target dependencies of the library
# as an example, code may need to be generated before libraries
# either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Declare a C++ executable
# With ament_cmake, all packages are built within a single CMake context
# The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/wheeltec_robot_gazebo_node.cpp)

# Rename C++ executable without prefix
# The above recommended prefix causes long target names, the following renames the
# target back to the shorter version for ease of user use
# e.g., "ros2 run someones_pkg node" instead of "ros2 run someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

# Add cmake target dependencies of the executable
# Same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

# Install executables and/or libraries
# ament_package() calls macros
# include and target_include_directories can be used to specify include paths
# See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# ament_export_dependencies(${catkin_EXPORTED_TARGETS})
ament_export_dependencies(
  gazebo_plugins
  gazebo_ros
  gazebo_ros_control
  rclcpp
  rclpy
  std_msgs
)
ament_export_include_directories(
  include
)
ament_package()
