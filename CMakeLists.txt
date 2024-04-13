cmake_minimum_required(VERSION 3.8)
project(sbus_serial)

# C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

# Find and dependencies for all node executables
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
set(dependencies "rclcpp" "std_msgs" "geometry_msgs")

# Generate the interfaces
find_package(rosidl_default_generators REQUIRED)
set(msg_files
  "msg/Sbus.msg"
  DEPENDENCIES std_msgs
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
ament_export_dependencies(rosidl_default_runtime)


# sbus_serial_node: include the generated interfaces
add_executable(sbus_serial_node src/sbus_serial_node.cpp src/sbus_serial_driver.cpp)
ament_target_dependencies(sbus_serial_node ${dependencies})
target_link_libraries(sbus_serial_node "${cpp_typesupport_target}")

# sbus_cmd_vel_node: include the generated interfaces
add_executable(sbus_cmd_vel_node src/sbus_cmd_vel_node.cpp)
ament_target_dependencies(sbus_cmd_vel_node ${dependencies})
target_link_libraries(sbus_cmd_vel_node "${cpp_typesupport_target}")

# Separate executable for the calibration
add_executable(sbus_calibrate src/sbus_calibrate.cpp src/sbus_serial_driver.cpp)

# Install executables in lib
install(
	TARGETS sbus_serial_node sbus_cmd_vel_node
	DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install calibration binary in share
install(
	TARGETS sbus_calibrate
	DESTINATION share/${PROJECT_NAME}
)

ament_package()
