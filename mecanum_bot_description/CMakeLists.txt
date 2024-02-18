cmake_minimum_required(VERSION 3.8)
project(mecanum_bot_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch mecanum_bot rviz worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks("mecanum_bot_description_gazebo_paths.dsv.in")

ament_package()
