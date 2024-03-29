cmake_minimum_required(VERSION 3.8)
project(plato_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(moveit_common REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

moveit_package()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  Boost
  control_msgs
  Eigen3
  interactive_markers
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_servo
  # moveit_task_constructor_core
  # moveit_visual_tools
  pluginlib
  rclcpp
  rclcpp_action
  # rviz_visual_tools
  tf2_geometry_msgs
  tf2_ros
  
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(${THIS_PACKAGE_INCLUDE_DIRS})

add_executable(controller_PlatoHand src/controller_PlatoHand.cpp)
add_executable(controller_spaceMouse src/controller_spaceMouse.cpp)

target_include_directories(controller_PlatoHand
    PUBLIC include)

target_include_directories(controller_spaceMouse
    PUBLIC include)

ament_target_dependencies(controller_PlatoHand
    ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)
ament_target_dependencies(controller_spaceMouse
    ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)


install(TARGETS controller_PlatoHand
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS controller_spaceMouse
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
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

ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_export_include_directories(include)


ament_package()