^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package gpsd_client
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.3 (2022-11-28)
------------------
* Adding better debugging output to help diagnose corner case (`#59 <https://github.com/swri-robotics/gps_umd/issues/59>`_)
  * Adding better debugging output to help diagnose corner case
  DISTRIBUTION A. Approved for public release; distribution unlimited. OPSEC `#4584 <https://github.com/swri-robotics/gps_umd/issues/4584>`_
* Merge pull request `#39 <https://github.com/swri-robotics/gps_umd/issues/39>`_ from shr-project/jansa/gpsd
  Fix build with gpsd-3.21
* Fix build with gpsd-3.21
  Adapt to changes from this commit:
  https://gitlab.com/gpsd/gpsd/-/commit/29991d6ffeb41ecfc8297db68bb68be0128c8514
* Contributors: David Anthony, Martin Jansa

0.3.2 (2020-05-25)
------------------
* Fix build with gpsd 3.20 (`#28 <https://github.com/swri-robotics/gps_umd/issues/28>`_)
* Contributors: P. J. Reed

0.3.1 (2020-03-05)
------------------
* Fix for gpsd-3.19 compatibility (`#26 <https://github.com/swri-robotics/gps_umd/issues/26>`_)
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
