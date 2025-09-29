cmake_minimum_required(VERSION 3.5)
project(epson_robot_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
