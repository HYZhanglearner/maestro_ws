cmake_minimum_required(VERSION 3.8)
project(udp_receiver)

# Add compile options, if using GNU C++ or Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set the path to the ASIO library
set(ASIO_PATH "${CMAKE_CURRENT_SOURCE_DIR}/asio-1.24.0/include")

# Add the ASIO include directories
include_directories(${ASIO_PATH})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Find the required OpenGL libraries
set(OpenGL_GL_PREFERENCE "GLVND") # or "LEGACY"
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

# Find Python Libraries
find_package(PythonLibs 3.10 REQUIRED)  # Adjust the version as per your system


# ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)


# Add executables
add_executable(UDPReceiver receiver_main.cpp UDP_receiver.cpp hand_display.cpp)
add_executable(UDPReceiver_mthread receiver_main_thread.cpp UDP_receiver.cpp hand_display.cpp)
add_executable(render_test render_test.cpp hand_display.cpp)

# Specify libraries to link a library or executable target against
target_link_libraries(UDPReceiver 
    pthread 
    ${OPENGL_LIBRARIES} 
    ${GLUT_LIBRARY} 
    ${PYTHON_LIBRARIES}
    ${rclcpp_LIBRARIES}
    ${std_msgs_LIBRARIES} 
)
target_link_libraries(UDPReceiver_mthread 
    pthread 
    ${OPENGL_LIBRARIES} 
    ${GLUT_LIBRARY} 
    ${PYTHON_LIBRARIES}
    ${rclcpp_LIBRARIES}
    ${std_msgs_LIBRARIES} 
)
target_link_libraries(render_test 
    ${OPENGL_LIBRARIES} 
    ${GLUT_LIBRARY} 
    ${PYTHON_LIBRARIES}
    ${rclcpp_LIBRARIES}
    ${std_msgs_LIBRARIES} 
)

# ROS 2 specific dependencies for ROS related executables
ament_target_dependencies(UDPReceiver
    rclcpp
    sensor_msgs
)
ament_target_dependencies(UDPReceiver_mthread
    rclcpp
    sensor_msgs
)

ament_target_dependencies(render_test
    rclcpp
    sensor_msgs
)


# Install executables
install(TARGETS
  UDPReceiver
  UDPReceiver_mthread
  render_test
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

