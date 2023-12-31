cmake_minimum_required(VERSION 3.8)
project(ic2d_urdf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(ic2d_effort_controller src/effort_controller.cpp)
ament_target_dependencies(ic2d_effort_controller rclcpp std_msgs)

add_executable(ic2d_position_controller src/position_controller.cpp)
ament_target_dependencies(ic2d_position_controller rclcpp std_msgs)

install(
  TARGETS
    ic2d_effort_controller
    ic2d_position_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY description launch meshes config world
  DESTINATION share/${PROJECT_NAME}
)

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
