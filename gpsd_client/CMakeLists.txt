cmake_minimum_required(VERSION 3.10)
project(gpsd_client)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)

find_package(gps_msgs REQUIRED)
find_package(gps_extended_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED rclcpp_components)
find_package(sensor_msgs REQUIRED)

# Try to find libgps, first with CMake's usual library search method, then by
# querying pkg-config.
find_library(libgps_LIBRARIES NAMES gps)
find_path(libgps_INCLUDE_DIRS NAMES libgpsmm.h gps.h)

if(NOT libgps_LIBRARIES)
  message(STATUS "Checking pkg-config for libgps")
  find_package(PkgConfig)
  if(PkgConfig_FOUND)
    pkg_check_modules(libgps libgps)
  endif()
endif()

if(NOT libgps_LIBRARIES)
  message(FATAL_ERROR "Could not find libgps "
                      "(hint for Debian/Ubuntu: apt install libgps-dev)")
else()
  message(STATUS "Found libgps: ${libgps_LIBRARIES}")
endif()

add_library(${PROJECT_NAME} SHARED
  src/client.cpp
  src/gpsd_parser_factory.cpp
  src/gpsd_raw_parser.cpp
  src/parsers/gpsd_parser_base.cpp)

# sensor_msgs 5.3.2 was the first version to define an unknown fix, so this flag is used to
# correctly initialize variables
if (sensor_msgs_VERSION VERSION_LESS "5.3.2")
  target_compile_definitions(${PROJECT_NAME} PRIVATE "-DNO_UNKNOWN_FIX")
endif()

target_include_directories(${PROJECT_NAME}
  PUBLIC
    include
    ${rclcpp_components_INCLUDE_DIRS}
    ${libgps_INCLUDE_DIRS}
    ${gps_msgs_INCLUDE_DIRS}
    ${gps_extended_msgs_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS})

target_compile_definitions(${PROJECT_NAME}
  PRIVATE "COMPOSITION_BUILDING_DLL")

rclcpp_components_register_nodes(${PROJECT_NAME} "gpsd_client::GPSDClientComponent")
target_link_libraries(${PROJECT_NAME} PUBLIC
  ${libgps_LIBRARIES}
  ${gps_msgs_TARGETS}
  ${gps_extended_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_components::component
  rclcpp_components::component_manager
  sensor_msgs::sensor_msgs_library
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  # Where libgps lives at runtime. When gpsd_client is pinned to a
  # source-built libgps (as tools/test_against_gpsd.sh does for each API
  # version), that directory is on no default search path, and `colcon test`
  # runs the test binaries with its own environment rather than the shell's --
  # so a test that passes when run by hand fails under colcon with
  # "libgps.so.NN: cannot open shared object file".
  #
  # APPEND_LIBRARY_DIRS is the ament-supported way to say this; it beats
  # relying on RUNPATH or on the caller exporting LD_LIBRARY_PATH.
  if(IS_ABSOLUTE "${libgps_LIBRARIES}")
    get_filename_component(libgps_LIBRARY_DIRS "${libgps_LIBRARIES}" DIRECTORY)
  endif()

  ament_add_gtest(test_gpsd_parser test/test_gpsd_parser.cpp
    APPEND_LIBRARY_DIRS "${libgps_LIBRARY_DIRS}")
  target_link_libraries(test_gpsd_parser ${PROJECT_NAME})

  # The JSON fixture harness mirrors gps_open()'s initialization, which calls
  # gps_clear_gst()/gps_clear_log(). Neither is available in every supported
  # libgps, and their availability does not track GPSD_API_MAJOR_VERSION --
  # gpsd 3.24 and 3.26.1 are both API 14.0, but only 3.26.1 declares
  # gps_clear_gst() -- so they have to be probed rather than version-guarded.
  include(CheckCXXSymbolExists)
  set(CMAKE_REQUIRED_INCLUDES ${libgps_INCLUDE_DIRS})
  check_cxx_symbol_exists(gps_clear_gst "gps.h" HAVE_GPS_CLEAR_GST)
  check_cxx_symbol_exists(gps_clear_log "gps.h" HAVE_GPS_CLEAR_LOG)
  unset(CMAKE_REQUIRED_INCLUDES)

  ament_add_gtest(test_gpsd_json_fixture
    test/test_gpsd_json_fixture.cpp
    test/gpsd_json_fixture.cpp
    APPEND_LIBRARY_DIRS "${libgps_LIBRARY_DIRS}")

  # Covers the generated fill code and the raw parser. Its first include is
  # <gps.h>, which also serves as the standing regression test that a
  # generated message header survives being parsed after gps.h -- the
  # collision D9's constant renaming exists to prevent.
  ament_add_gtest(test_gpsd_raw_parser
    test/test_gpsd_raw_parser.cpp
    test/gpsd_json_fixture.cpp
    APPEND_LIBRARY_DIRS "${libgps_LIBRARY_DIRS}")
  target_link_libraries(test_gpsd_raw_parser ${PROJECT_NAME})

  # Its own target because it must include <gps.h> before anything else, and
  # so must not link anything that reaches the legacy GPSFix/GPSStatus headers.
  ament_add_gtest(test_gpsd_raw_include_order
    test/test_gpsd_raw_include_order.cpp
    APPEND_LIBRARY_DIRS "${libgps_LIBRARY_DIRS}")
  target_link_libraries(test_gpsd_raw_include_order ${PROJECT_NAME})
  if(HAVE_GPS_CLEAR_GST)
    target_compile_definitions(test_gpsd_raw_parser PRIVATE HAVE_GPS_CLEAR_GST)
  endif()
  if(HAVE_GPS_CLEAR_LOG)
    target_compile_definitions(test_gpsd_raw_parser PRIVATE HAVE_GPS_CLEAR_LOG)
  endif()
  target_link_libraries(test_gpsd_json_fixture ${PROJECT_NAME})
  if(HAVE_GPS_CLEAR_GST)
    target_compile_definitions(test_gpsd_json_fixture PRIVATE HAVE_GPS_CLEAR_GST)
  endif()
  if(HAVE_GPS_CLEAR_LOG)
    target_compile_definitions(test_gpsd_json_fixture PRIVATE HAVE_GPS_CLEAR_LOG)
  endif()

  # Tier 2: gpsfake -> real gpsd -> this node -> ROS topics.
  #
  # Skips itself unless pointed at a gpsd built with the daemon and Python
  # module (GPSD_TIER2_PREFIX) and a source checkout for the log corpus
  # (GPSD_REPO), neither of which exists on the build farm. See the test's
  # docstring for how to run it.
  #
  # GPSD_EXPECTED_RAW_MSG is computed here because this is the only place that
  # knows which gps.h was compiled against; the test uses it to assert the
  # node advertised the message for *this* API version rather than merely
  # advertising something. Read from the header rather than from the installed
  # library version: a pair spans a range of releases, so the release number
  # does not determine the API pair.
  find_file(GPSD_HEADER gps.h PATHS ${libgps_INCLUDE_DIRS} NO_DEFAULT_PATH)
  if(GPSD_HEADER)
    file(STRINGS "${GPSD_HEADER}" _api_major_line
      REGEX "^#define[ \t]+GPSD_API_MAJOR_VERSION[ \t]+[0-9]+")
    file(STRINGS "${GPSD_HEADER}" _api_minor_line
      REGEX "^#define[ \t]+GPSD_API_MINOR_VERSION[ \t]+[0-9]+")
    string(REGEX MATCH "[0-9]+$" _api_major "${_api_major_line}")
    string(REGEX MATCH "[0-9]+$" _api_minor "${_api_minor_line}")
  endif()

  if(_api_major AND NOT _api_minor STREQUAL "")
    ament_add_pytest_test(test_gpsfake_end_to_end
      "${CMAKE_CURRENT_SOURCE_DIR}/test/test_gpsfake_end_to_end.py"
      TIMEOUT 900
      ENV "GPSD_EXPECTED_RAW_MSG=GPSDRaw${_api_major}v${_api_minor}")
  else()
    # Not fatal: every other suite still runs. But say so, because a silently
    # missing end-to-end test is worse than a noisy one.
    message(WARNING
      "Could not read GPSD_API_*_VERSION from '${GPSD_HEADER}'; "
      "skipping the gpsfake end-to-end test")
  endif()
endif()

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
