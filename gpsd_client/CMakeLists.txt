cmake_minimum_required(VERSION 3.10)
project(gpsd_client)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)

find_package(gps_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules (libgps REQUIRED libgps)

add_library(${PROJECT_NAME} SHARED
  src/client.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
  ${libgps_INCLUDE_DIRS}
)
target_compile_definitions(${PROJECT_NAME}
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
rclcpp_components_register_nodes(${PROJECT_NAME} "gpsd_client::GPSDClientComponent")
target_link_libraries(${PROJECT_NAME}
  ${libgps_LIBRARIES}
)
ament_target_dependencies(${PROJECT_NAME}
  "gps_msgs"
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/)

install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}/)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_libraries(${PROJECT_NAME})

ament_package()
