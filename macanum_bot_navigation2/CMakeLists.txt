cmake_minimum_required(VERSION 3.5)
project(mecanum_bot_navigation2)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch map param rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
