cmake_minimum_required(VERSION 3.14)
project(override_param)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(main
  main.cpp
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
