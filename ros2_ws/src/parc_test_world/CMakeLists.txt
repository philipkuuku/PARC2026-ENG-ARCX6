cmake_minimum_required(VERSION 3.8)
project(parc_test_world)

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch models worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
