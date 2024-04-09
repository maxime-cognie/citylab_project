cmake_minimum_required(VERSION 3.8)
project(robot_patrol)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(direction_interface REQUIRED)

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

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetDirection.srv"
  DEPENDENCIES sensor_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp
)

add_executable(robot_patrol_node src/patrol.cpp)
ament_target_dependencies(robot_patrol_node rclcpp sensor_msgs geometry_msgs)

add_executable(direction_service_node src/direction_service.cpp)
ament_target_dependencies(direction_service_node rclcpp sensor_msgs direction_interface)
target_link_libraries(direction_service_node "${cpp_typesupport_target}")

add_executable(robot_patrol_service_node src/patrol_with_service.cpp)
ament_target_dependencies(robot_patrol_service_node rclcpp sensor_msgs geometry_msgs)
target_link_libraries(robot_patrol_service_node "${cpp_typesupport_target}")

install(TARGETS
	robot_patrol_node
	direction_service_node
	robot_patrol_service_node
	DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
	launch
	DESTINATION share/${PROJECT_NAME}
)

ament_package()
