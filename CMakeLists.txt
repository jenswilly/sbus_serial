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
find_package(sbus_interfaces REQUIRED)

set(dependencies "rclcpp" "std_msgs" "geometry_msgs" "sbus_interfaces")

# Add executable and link dependencies
add_executable(sbus_serial src/sbus_serial_node.cpp src/sbus_serial_driver.cpp)
ament_target_dependencies(sbus_serial ${dependencies})

add_executable(sbus_test_publisher src/sbus_test_publisher.cpp)
ament_target_dependencies(sbus_test_publisher ${dependencies})

add_executable(sbus_cmd_vel src/sbus_cmd_vel_node.cpp)
ament_target_dependencies(sbus_cmd_vel ${dependencies})

# Separate executable for the calibration
add_executable(sbus_calibrate src/sbus_calibrate.cpp src/sbus_serial_driver.cpp)

# Install executables in lib
install(
	TARGETS 
		sbus_serial 
		sbus_test_publisher
		sbus_cmd_vel
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

# --- Old, unconverted stuff below ---

## Find catkin macros and libraries
#find_package(catkin REQUIRED COMPONENTS
#	roscpp
#    std_msgs
#    geometry_msgs
#    message_generation
#)


# Generate messages in the 'msg' folder
#add_message_files(
#   FILES
#   Sbus.msg
#)

# Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#  std_msgs
#)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sbus_serial
#	CATKIN_DEPENDS roscpp std_msgs
#	DEPENDS pthread
#)

#include_directories(
#	include
#	${catkin_INCLUDE_DIRS}
#)

# # sbus_node
# add_executable( ${PROJECT_NAME}_node
# 	src/sbus_serial_driver.cpp
# 	src/sbus_serial_node.cpp )
# set_target_properties( ${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME sbus_serial_node PREFIX "")
# target_link_libraries( ${PROJECT_NAME}_node pthread ${catkin_LIBRARIES} )
# add_dependencies(
# 	${PROJECT_NAME}_node
# 	${${PROJECT_NAME}_EXPORTED_TARGETS}
# 	${catkin_EXPORTED_TARGETS}
# )

# # sbus_cmd_vel_node
# add_executable( sbus_cmd_vel_node src/sbus_cmd_vel_node.cpp )
# target_link_libraries( sbus_cmd_vel_node ${catkin_LIBRARIES} )
# add_dependencies(
# 	sbus_cmd_vel_node
# 	${${PROJECT_NAME}_EXPORTED_TARGETS}
# 	${catkin_EXPORTED_TARGETS}
# )

# # sbus_calibrate test app (not ROS node)
# add_executable( sbus_calibrate
# 	src/sbus_serial_driver.cpp
# 	src/sbus_calibrate.cpp )
# target_link_libraries( sbus_calibrate pthread ${catkin_LIBRARIES} )
# add_dependencies(
# 	${PROJECT_NAME}_node
# 	${${PROJECT_NAME}_EXPORTED_TARGETS}
# 	${catkin_EXPORTED_TARGETS}
# )
