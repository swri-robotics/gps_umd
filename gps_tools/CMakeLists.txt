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

# Install the python module for this package
ament_python_install_package(${PROJECT_NAME})

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
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_components::component
)

install(TARGETS
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
  # Uncrustify and cppcheck would read conversions.h as C, which cannot parse
  # its namespace.
  set(ament_cmake_cppcheck_LANGUAGE c++)
  set(ament_cmake_uncrustify_ADDITIONAL_ARGS LANGUAGE CPP)
  # conversions.h is Chuck Gantz's UTM code by way of utexas-art-ros-pkg. It
  # keeps its original attribution, which names no license, so it cannot carry
  # this package's license header. ament_copyright skips it, and cpplint skips
  # its copyright check, which ament_copyright covers for every other file.
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright ament_cmake_cpplint)
  ament_lint_auto_find_test_dependencies()
  ament_copyright(EXCLUDE include/gps_tools/conversions.h)
  ament_cpplint(FILTERS -legal/copyright)

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_conversions test/test_conversions.cpp)
  target_include_directories(test_conversions PRIVATE include)

  # Loads both components from their libraries, as a container would.
  find_package(class_loader REQUIRED)
  ament_add_gtest(test_utm_components test/test_utm_components.cpp TIMEOUT 120)
  target_include_directories(test_utm_components PRIVATE include)
  target_link_libraries(test_utm_components
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

  # Starts the installed fix_translator through its launch file.
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_fix_translator_launch.py TIMEOUT 120)
endif()

ament_export_include_directories(include)
ament_export_libraries(utm_odometry_component utm_odometry_to_navsatfix_component)
ament_export_dependencies(ament_cmake)

ament_package()
