gps_umd [![Build Status](https://travis-ci.org/swri-robotics/gps_umd.svg?branch=dashing-devel)](https://travis-ci.org/swri-robotics/gps_umd)
=======

This package contains messages for representing data from GPS devices and algorithms for manipulating it.

This branch converts the messages and algorithms in this repository to support ROS 2 Dashing.

There have been a few architectural changes; if you were using these packages in ROS1, note that the `gps_common` package has been split into two packages: `gps_msgs` contains only message definitions, and `gps_tools` contains the nodes and scripts that were in `gps_common`.  In addition, all of the C++ nodes that were in this repository have been converted into Components.

NavSatFix vs. GPSFix
--------------------

The node `fix_translator` converts [sensor_msgs/NavSatFix](http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html) messages to [gps_common/GPSFix](http://docs.ros.org/api/gps_common/html/msg/GPSFix.html) messages and vice versa. Usage examples:

### Translate from NavSatFix to GPSFix

```xml
  <node name="fix_translator" pkg="gps_common" type="fix_translator">
    <!-- Translate from NavSatFix to GPSFix //-->
      <remap from="/navsat_fix_in"  to="/YOUR_NAVSATFIX_TOPIC"/>
      <remap from="/gps_fix_out"    to="/YOUR_GPSFIX_TOPIC"/>
  </node>
```


### Translate from GPSFix to NavSatFix

```xml
  <node name="fix_translator" pkg="gps_common" type="fix_translator">
    <!-- Translate from GPSFix to NavSatFix //-->
       <remap from="/gps_fix_in"     to="/YOUR_GPSFIX_TOPIC"/>
       <remap from="/navsat_fix_out" to="/YOUR_NAVSATFIX_TOPIC"/>
  </node>
```

Only adjust the topic names after "to=" in each remap line.

Use with ros1_bridge
--------------------------------

The [ros1_bridge](https://index.ros.org/p/ros1_bridge/) package must be built from source to enable playback of GPSFix and GPSStatus messages stored in ROS1 bags. This requires that the applicable ROS1 `gps_common` and ROS2 `gps_msgs` packages are first installed.
