^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package gpsd_client
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.5 (2025-05-20)
------------------
* Removing all ament_target_dependencies() calls (`#111 <https://github.com/swri-robotics/gps_umd/issues/111>`_)
* Changing hard coded default port value and improving string allocation (`#101 <https://github.com/swri-robotics/gps_umd/issues/101>`_)
* Updating package search (`#99 <https://github.com/swri-robotics/gps_umd/issues/99>`_)
  * Updating package search
  * Removing pkg_check_modules call
* Check mode Field (`#100 <https://github.com/swri-robotics/gps_umd/issues/100>`_)
  * Switching to use mode of fix instead of status to be more robust to changes in API
  * Making altitude NaN if in 2D fix mode
* Port of https://github.com/swri-robotics/gps_umd/pull/74 (`#98 <https://github.com/swri-robotics/gps_umd/issues/98>`_)
* Contributors: David Anthony

2.0.4 (2024-05-07)
------------------
* Fix queue build up issue in gpsd_client (`#89 <https://github.com/swri-robotics/gps_umd/issues/89>`_)
* Updating CI process (`#85 <https://github.com/swri-robotics/gps_umd/issues/85>`_)
* Contributors: David Anthony, Erik Botö

2.0.3 (2023-09-06)
------------------
* Fix uninitialized parameter exception in gpsd_client (`#83 <https://github.com/swri-robotics/gps_umd/issues/83>`_)
* Contributors: Erik Botö

2.0.2 (2023-06-23)
------------------
* declare host and port parameters (`#80 <https://github.com/swri-robotics/gps_umd/issues/80>`_)
* Contributors: Adam Aposhian

2.0.1 (2023-06-08)
------------------

1.0.7 (2023-04-04)
------------------

1.0.6 (2023-04-04)
------------------

1.0.5 (2022-08-30)
------------------
* Cleaner shutdown after unload
* Fix build issues with gpsd 3.21 and 3.23
* Fixing build warnings about deprecated API. DISTRIBUTION A. Approved for public release; distribution unlimited. OPSEC `#4584 <https://github.com/swri-robotics/gps_umd/issues/4584>`_ (`#61 <https://github.com/swri-robotics/gps_umd/issues/61>`_)
* Adding debug message to help diagnose failures (`#60 <https://github.com/swri-robotics/gps_umd/issues/60>`_)
* User configurable publish rate (`#58 <https://github.com/swri-robotics/gps_umd/issues/58>`_)
 * Add demo launch file
* Fix ros2 component topics (`#46 <https://github.com/swri-robotics/gps_umd/issues/46>`_)
* Contributors: Dave Mohamad, David Anthony, Philip Cheney

1.0.4 (2020-08-14)
------------------

1.0.3 (2020-06-10)
------------------
* Foxy support (`#29 <https://github.com/swri-robotics/gps_umd/issues/29>`_)
* Contributors: P. J. Reed

1.0.2 (2020-03-05)
------------------
* Fix for gpsd-3.19 compatibility (`#26 <https://github.com/swri-robotics/gps_umd/issues/26>`_)
* Contributors: P. J. Reed

1.0.1 (2020-03-05)
------------------

1.0.0 (2019-10-04)
------------------
* Support ROS2 (`#24 <https://github.com/pjreed/gps_umd/issues/24>`_)
* Contributors: P. J. Reed

0.3.0 (2019-10-03)
------------------

0.2.0 (2017-11-16)
------------------
* Add include for <cmath> in gpsd_client
* Add parameter to set frame_id.
* Contributors: Kris Kozak, P. J. Reed

0.1.9 (2017-05-08)
------------------

0.1.8 (2016-10-31)
------------------
* Use pre-processor defines to handle different libgps API versions
  Fixes `#1 <https://github.com/swri-robotics/gps_umd/issues/1>`_
* Contributors: P. J. Reed

0.1.7 (2014-05-08)
------------------
* Fix a segfault when there is no GPS fix: time will be NaN which causes the ROS timestamp message to throw a Boost rounding exception.
* Contributors: Stuart Alldritt

0.1.6
-----
* Initial catkin release
