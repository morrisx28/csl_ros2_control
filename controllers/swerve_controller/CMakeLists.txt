cmake_minimum_required(VERSION 3.14)
project(swerve_controller)

# Enable C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Torch (PyTorch C++ API)
find_package(Torch REQUIRED)

# YAML CPP
find_package(yaml-cpp REQUIRED)

include_directories(
  include
)

add_library(${PROJECT_NAME} SHARED
  src/swerve_controller.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  controller_interface
  hardware_interface
  pluginlib
  std_msgs
  geometry_msgs
  yaml-cpp
)

target_link_libraries(${PROJECT_NAME} 
  yaml-cpp 
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES
  resource/swerve_controller.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(controller_interface resource/swerve_controller.xml)

ament_package()
