cmake_minimum_required(VERSION 3.8)
project(robot_patrol)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(rosidl_typesupport_c REQUIRED)

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
  "action/GoToPose.action"
  DEPENDENCIES sensor_msgs geometry_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp
)

add_executable(robot_patrol_node src/patrol.cpp)
ament_target_dependencies(robot_patrol_node rclcpp sensor_msgs geometry_msgs)

add_executable(direction_service_node src/direction_service.cpp)
ament_target_dependencies(direction_service_node rclcpp sensor_msgs)
target_link_libraries(direction_service_node "${cpp_typesupport_target}")

add_executable(test_service_node src/test_service.cpp)
ament_target_dependencies(test_service_node rclcpp sensor_msgs)
target_link_libraries(test_service_node "${cpp_typesupport_target}")

add_executable(robot_patrol_service_node src/patrol_with_service.cpp)
ament_target_dependencies(robot_patrol_service_node rclcpp sensor_msgs geometry_msgs)
target_link_libraries(robot_patrol_service_node "${cpp_typesupport_target}")

add_executable(go_to_pose_as_node src/go_to_pose_action.cpp)
ament_target_dependencies(go_to_pose_as_node rclcpp rclcpp_action geometry_msgs nav_msgs tf2)
target_link_libraries(go_to_pose_as_node "${cpp_typesupport_target}")

add_executable(test_action src/test_action.cpp)
ament_target_dependencies(test_action rclcpp rclcpp_action)
target_link_libraries(test_action "${cpp_typesupport_target}")

install(TARGETS
	robot_patrol_node
	direction_service_node
	test_service_node
	robot_patrol_service_node
	go_to_pose_as_node
	test_action
	DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
	launch
	DESTINATION share/${PROJECT_NAME}
)

ament_package()
