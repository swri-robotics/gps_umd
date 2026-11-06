cmake_minimum_required(VERSION 3.14.4)
project(gps_tools)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

# Debian and Ubuntu ship GeographicLib with only a find module, outside CMake's
# default search path. Where it is absent, as on RHEL, find_package falls back
# to GeographicLib's own package config. Both set the variables used below.
list(APPEND CMAKE_MODULE_PATH /usr/share/cmake/geographiclib)
find_package(GeographicLib REQUIRED)

# Install the python module for this package
ament_python_install_package(${PROJECT_NAME})

# The UTM conversions in gps_tools/conversions.hpp. GeographicLib is private to
# this library, so packages using the header need neither its headers nor a
# way to find it.
add_library(gps_tools_conversions SHARED
  src/conversions.cpp)
target_include_directories(gps_tools_conversions
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_include_directories(gps_tools_conversions SYSTEM PRIVATE ${GeographicLib_INCLUDE_DIRS})
target_link_libraries(gps_tools_conversions PRIVATE ${GeographicLib_LIBRARIES})
# The header carries no export macros.
set_target_properties(gps_tools_conversions PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)

add_library(utm_odometry_component SHARED
  src/utm_odometry_component.cpp)
target_include_directories(utm_odometry_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_definitions(utm_odometry_component
  PRIVATE "COMPOSITION_BUILDING_DLL")

rclcpp_components_register_nodes(utm_odometry_component "gps_tools::UtmOdometryComponent")
target_link_libraries(utm_odometry_component PRIVATE
  gps_tools_conversions
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_components::component
)

add_library(utm_odometry_to_navsatfix_component SHARED
  src/utm_odometry_to_navsatfix_component.cpp)

target_include_directories(utm_odometry_to_navsatfix_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_definitions(utm_odometry_to_navsatfix_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
rclcpp_components_register_nodes(utm_odometry_to_navsatfix_component "gps_tools::UtmOdometryToNavSatFixComponent")
target_link_libraries(utm_odometry_to_navsatfix_component PRIVATE
  gps_tools_conversions
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_components::component
)

install(TARGETS
    gps_tools_conversions
    utm_odometry_component
    utm_odometry_to_navsatfix_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)
# ros2 run and launch_ros look for a package's executables in lib/<package>.
install(PROGRAMS
    scripts/bag_converter
    nodes/fix_translator
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # cppcheck parses .h files as C, which fails on the C++ that the deprecated
  # conversions.h pulls in from conversions.hpp.
  set(ament_cmake_cppcheck_LANGUAGE "c++")
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_conversions test/test_conversions.cpp)
  target_link_libraries(test_conversions gps_tools_conversions)

  # The deprecated conversions.h must keep compiling and forwarding.
  ament_add_gtest(test_conversions_deprecated_header test/test_conversions_deprecated_header.cpp)
  target_link_libraries(test_conversions_deprecated_header gps_tools_conversions)

  # Loads both components from their libraries, as a container would.
  find_package(class_loader REQUIRED)
  ament_add_gtest(test_utm_components test/test_utm_components.cpp TIMEOUT 120)
  target_link_libraries(test_utm_components
    gps_tools_conversions
    class_loader::class_loader
    ${nav_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    rclcpp::rclcpp
    rclcpp_components::component)
  target_compile_definitions(test_utm_components PRIVATE
    UTM_ODOMETRY_LIBRARY="$<TARGET_FILE:utm_odometry_component>"
    UTM_ODOMETRY_TO_NAVSATFIX_LIBRARY="$<TARGET_FILE:utm_odometry_to_navsatfix_component>")
  add_dependencies(test_utm_components
    utm_odometry_component utm_odometry_to_navsatfix_component)

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_gps_message_converter test/test_gps_message_converter.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
    TIMEOUT 120)

  # Stops the installed fix_translator with SIGINT and SIGTERM.
  ament_add_pytest_test(test_bag_converter test/test_bag_converter.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
    TIMEOUT 120)

  ament_add_pytest_test(test_fix_translator_shutdown test/test_fix_translator_shutdown.py
    TIMEOUT 120)

  # Starts the installed fix_translator through its launch file.
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_fix_translator_launch.py TIMEOUT 120)
endif()

ament_export_include_directories(include)
ament_export_libraries(
  gps_tools_conversions utm_odometry_component utm_odometry_to_navsatfix_component)
ament_export_dependencies(ament_cmake)

ament_package()
