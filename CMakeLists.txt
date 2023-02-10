cmake_minimum_required(VERSION 3.5)
project(stage_ros2)

# Default to C99
if (NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

if ($ENV{ROS_DISTRO} STREQUAL "galactic" OR $ENV{ROS_DISTRO} STREQUAL "foxy")
  add_compile_options(-DUSE_LEGACY_MSGS_INCLUDE)
endif ()

# dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mrpt_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# We must use lowercase Stage as the argument for the find_package below.
# Both works but they result in different variable being set `Stage_DIR` vs `stage_DIR`,
# and variable names are case-sensitive. Stage's stage-config.cmake file incorrectly uses ${stage_DIR}.
find_package(stage REQUIRED)

# build
add_executable(stage_ros2 src/stage_ros2.cc)
# Add Stage dependency (the STAGE_* variables are defined by stage-config.cmake),
# which was found by `find_package(stage REQUIRED)`.
target_include_directories(stage_ros2
		PRIVATE ${STAGE_INCLUDE_DIRS} include)
target_link_libraries(stage_ros2
		PRIVATE ${STAGE_LIBRARIES})
# Add dependencies on standard ROS 2 packages using ament.
ament_target_dependencies(stage_ros2
		PUBLIC
		rclcpp
		rcutils
		cv_bridge
		image_transport
		geometry_msgs
		nav_msgs
		sensor_msgs
		mrpt_msgs
		std_msgs
		std_srvs
		tf2
		tf2_ros
		tf2_geometry_msgs)

install(TARGETS stage_ros2
		DESTINATION lib/${PROJECT_NAME})

ament_package()
