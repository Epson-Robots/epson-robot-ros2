cmake_minimum_required(VERSION 3.8)
project(epson_ros2_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(joint_trajectory_controller REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)


add_library(epson_joint_trajectory_controller SHARED src/epson_joint_trajectory_controller.cpp)
target_include_directories(epson_joint_trajectory_controller PRIVATE include)
ament_target_dependencies(epson_joint_trajectory_controller controller_interface joint_trajectory_controller pluginlib rclcpp hardware_interface controller_manager rclcpp_lifecycle )

pluginlib_export_plugin_description_file(controller_interface epson_joint_trajectory_controller.xml)

install(TARGETS epson_joint_trajectory_controller
  DESTINATION lib
)

install(FILES epson_joint_trajectory_controller.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(epson_joint_trajectory_controller)

ament_package()
