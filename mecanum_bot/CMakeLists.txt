cmake_minimum_required(VERSION 3.5)
project(mecanum_bot)

# find dependencies
find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
