cmake_minimum_required(VERSION 3.14.4)
project(gps_umd)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
