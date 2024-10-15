cmake_minimum_required(VERSION 3.22)
project(my_ur5e_description)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

install(
  DIRECTORY launch urdf rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
