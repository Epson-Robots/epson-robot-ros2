cmake_minimum_required(VERSION 3.5)
project(epson_robot_control)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(epson_robot_msgs REQUIRED)

add_library(epson_robot_hardware_interface
  SHARED
  src/epson_robot_driver.cpp
  src/epson_robot_hardware_interface.cpp
)

target_link_libraries(epson_robot_hardware_interface ~/rtmc-client-library/build/librtmcclient.so)
target_include_directories(epson_robot_hardware_interface
                            PRIVATE
                            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                            $<INSTALL_INTERFACE:include> 
                            ~/rtmc-client-library/include)


ament_target_dependencies(epson_robot_hardware_interface
  rclcpp
  hardware_interface
  epson_robot_msgs
)


pluginlib_export_plugin_description_file(hardware_interface epson_robot_hardware_interface.xml)

install(
  TARGETS epson_robot_hardware_interface
  DESTINATION lib
)

ament_package()
