cmake_minimum_required(VERSION 3.8)
project(hand_pattern_recog)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Add executables
add_executable(hand_pattern_recog src/hand_pattern_recog.cpp
                src/finger.cpp
                )

# Include the Eigen directories
include_directories(${EIGEN3_INCLUDE_DIR})

# Include directories for your custom header files (Finger.h)
target_include_directories(hand_pattern_recog PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include/hand_pattern_recog
)

target_link_libraries(hand_pattern_recog
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES} 
  Eigen3::Eigen
)

# ROS 2 specific dependencies for ROS related executables
ament_target_dependencies(hand_pattern_recog
    rclcpp
    sensor_msgs
)

# Install executables
install(TARGETS
  hand_pattern_recog
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
