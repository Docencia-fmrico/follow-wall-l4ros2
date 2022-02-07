cmake_minimum_required(VERSION 3.5)
project(follow-wall-l4ros2)

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
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(example_interfaces REQUIRED)
  
set(dependencies
	rclcpp
  rclcpp_lifecycle
  std_msgs
  sensor_msgs
  example_interfaces
)

include_directories(include)

install(DIRECTORY include/
  DESTINATION include/
)

add_library(${PROJECT_NAME} SHARED
  src/follow-wall-l4ros2/SensingNode.cpp
  src/follow-wall-l4ros2/ActuationNode.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})


add_executable(follow_wall src/follow_wall_exec.cpp)
ament_target_dependencies(follow_wall ${dependencies})

add_executable(test_sensing src/test_sensing_node.cpp)
ament_target_dependencies(test_sensing ${dependencies})


target_link_libraries(follow_wall ${PROJECT_NAME})

target_link_libraries(test_sensing ${PROJECT_NAME})




install(TARGETS
	follow_wall
  test_sensing
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION lib/${PROJECT_NAME}
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

ament_package()
