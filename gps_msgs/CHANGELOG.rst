^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package gps_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.3 (2023-09-06)
------------------

2.0.2 (2023-06-23)
------------------

2.0.1 (2023-06-08)
------------------

1.0.7 (2023-04-04)
------------------

1.0.6 (2023-04-04)
------------------

1.0.5 (2022-08-30)
------------------

1.0.4 (2020-08-14)
------------------
* Add support for ros1 bridge (`#32 <https://github.com/swri-robotics/gps_umd/issues/32>`_)
* Contributors: Andrew Palmer

1.0.3 (2020-06-10)
------------------

1.0.2 (2020-03-05)
------------------

1.0.1 (2020-03-05)
------------------

1.0.0 (2019-10-04)
------------------
* Support ROS2 (`#24 <https://github.com/pjreed/gps_umd/issues/24>`_)
* Contributors: P. J. Reed

0.3.0 (2019-10-03)
------------------
* Switching order of commands to make catkin happy
* Initial attempt at creating an offline bag converter
* Changing ROS_INFO to ROS_DEBUG_THROTTLE as per `#15 <https://github.com/pjreed/gps_umd/issues/15>`_
* Update node name; make appending zone optional; add param for adding zone when going back to navsatfix
* Fix altitude, covariance
* Add reverse_utm_odometry_node
* Contributors: David Anthony, Dheera Venkatraman, P. J. Reed, Tim Clephas, Vikrant Shah, dheera

0.2.0 (2017-11-16)
------------------
* fix xml comments
* add install instructions and documentation
* add node for translating between NavSatFix and GPSFix
* Contributors: Andre Volk, P. J. Reed

0.1.9 (2017-05-08)
------------------
* removed unused variables
* Contributors: Frank Hoeller

0.1.8 (2016-10-31)
------------------
* Fixing orientation in UTM odometry message
  Before the fix, the orientation in the odometry message was set to x = 1, y = 0, z = 0, w = 0. This corresponds to Euler angles of roll = pi, pitch = 0, yaw = 0. I believe the intent was for the orientation to be the identity, which is x = 0, y = 0, z = 0, w = 1.
* Contributors: Tom Moore

0.1.6
-----
* Initial catkin release
