cmake_minimum_required(VERSION 3.5)
project(mecanum_bot_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch map rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
