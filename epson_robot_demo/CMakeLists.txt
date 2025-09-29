cmake_minimum_required(VERSION 3.8)
project(epson_robot_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(epson_robot_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(hardware_interface REQUIRED)

# epson_moveit_demo
add_executable(
  epson_moveit_demo
  src/epson_moveit_demo/demo.cpp
  src/epson_moveit_demo/epson_moveit_demo.cpp
  src/epson_robot_pickplace_point.cpp
)
target_include_directories(
  epson_moveit_demo
  PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ~/rtmc-client-library/include
)

ament_target_dependencies(epson_moveit_demo
  rclcpp
  moveit_ros_planning_interface
  epson_robot_msgs
)

install(TARGETS epson_moveit_demo
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)


# epson_moveit_except_demo
add_executable(
  epson_moveit_exception_demo
  src/epson_moveit_exception_demo/demo.cpp
  src/epson_moveit_exception_demo/epson_moveit_exception_demo.cpp
  src/epson_robot_pickplace_point.cpp
)
target_include_directories(
  epson_moveit_exception_demo
  PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ~/rtmc-client-library/include
)

ament_target_dependencies(epson_moveit_exception_demo
  rclcpp
  moveit_ros_planning_interface
  epson_robot_msgs
  controller_manager_msgs
  hardware_interface 
)

install(TARGETS epson_moveit_exception_demo
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
