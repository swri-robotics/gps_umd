^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package gpsd_client
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.9 (2023-06-08)
------------------

1.0.8 (2023-04-04)
------------------
* Updating authors and maintainers
* Contributors: David Anthony

1.0.7 (2023-04-04)
------------------
* Fixing Exports (`#73 <https://github.com/swri-robotics/gps_umd/issues/73>`_)
* Contributors: David Anthony

1.0.6 (2022-09-23)
------------------
* Fixing Foxy regression (`#67 <https://github.com/swri-robotics/gps_umd/issues/67>`_)
  Addresses https://github.com/swri-robotics/gps_umd/issues/66
* Contributors: David Anthony

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
* Updating changelogs
* Ros2 devel for GPSD 3.21 and 3.22 (`#65 <https://github.com/danthony06/gps_umd/issues/65>`_)
  * Better defaults
  * cleaner shutdown after unload
  * fix build issues with gpsd 3.21 and 3.23
  * Revert "fix build issues with gpsd 3.21 and 3.23"
  This reverts commit eaa7c7870696757d33224a2b969544560552583c.
  * fix build issues with gpsd 3.21 and 3.23
  * Revert "Better defaults"
  This reverts commit c0b66d610128cdcd9855585d8b67055d06e015c8.
* Fixing build warnings about deprecated API. DISTRIBUTION A. Approved for public release; distribution unlimited. OPSEC `#4584 <https://github.com/danthony06/gps_umd/issues/4584>`_ (`#61 <https://github.com/danthony06/gps_umd/issues/61>`_)
* Adding debug message to help diagnose failures (`#60 <https://github.com/danthony06/gps_umd/issues/60>`_)
* ros2-devel: user configurable publish rate (`#58 <https://github.com/danthony06/gps_umd/issues/58>`_)
  * Add user configurable publish rate
  * Add demo launch file
  * declare parameters
  * Update gpsd_client-launch.py
  Remove license, use project license
* Fix ros2 component topics (`#46 <https://github.com/danthony06/gps_umd/issues/46>`_)
  * Fix ros2 component topics
  Emit gps/navsat fix messages on timer tick.
  * Update gpsd_client/src/client.cpp
  Co-authored-by: Sivert Havso <sivert@havso.net>
  Co-authored-by: David Anthony <djanthony@gmail.com>
  Co-authored-by: Sivert Havso <sivert@havso.net>
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
