cmake_minimum_required(VERSION 3.8)
project(gofa_mujoco_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config urdf description meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
