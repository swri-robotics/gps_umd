gps_umd 
=======

This package contains messages for representing data from GPS devices and algorithms for manipulating it.

This branch converts the messages and algorithms in this repository to support ROS 2 Dashing.

There have been a few architectural changes; if you were using these packages in ROS1, note that the `gps_common` package has been split into two packages: `gps_msgs` contains only message definitions, and `gps_tools` contains the nodes and scripts that were in `gps_common`.  In addition, all of the C++ nodes that were in this repository have been converted into Components.

Build Status
--------
ROS2 Distro | Branch | Build status | Released packages
:---------: | :----: | :----------: | :---------------:
**Humble** | [`humble`](https://github.com/swri-robotics/gps_umd/tree/ros2-devel) | [![CI](https://github.com/swri-robotics/gps_umd/actions/workflows/main.yml/badge.svg?branch=ros2-devel)](https://github.com/swri-robotics/gps_umd/blob/ros2-devel/.github/workflows/main.yml?branch=ros2-devel) <br /> [![ROS2 Build Farm](http://build.ros2.org/buildStatus/icon?job=Hdev__gps_umd__ubuntu_jammy_amd64)](https://build.ros2.org/job/Hdev__gps_umd__ubuntu_jammy_amd64/) | [gps_msgs](https://index.ros.org/p/gps_msgs/#humble) <br /> [gps_tools](https://index.ros.org/p/gps_tools/#humble) <br /> [gpsd_client](https://index.ros.org/p/gpsd_client/#humble) 
**Jazzy** | [`jazzy`](https://github.com/swri-robotics/gps_umd/tree/ros2-devel) | [![CI](https://github.com/swri-robotics/gps_umd/actions/workflows/main.yml/badge.svg?branch=ros2-devel)](https://github.com/swri-robotics/gps_umd/blob/ros2-devel/.github/workflows/main.yml?branch=ros2-devel) <br /> [![ROS2 Build Farm](http://build.ros2.org/buildStatus/icon?job=Jdev__gps_umd__ubuntu_noble_amd64)](https://build.ros2.org/job/Jdev__gps_umd__ubuntu_noble_amd64/) | [gps_msgs](https://index.ros.org/p/gps_msgs/#jazzy) <br /> [gps_tools](https://index.ros.org/p/gps_tools/#jazzy) <br /> [gpsd_client](https://index.ros.org/p/gpsd_client/#jazzy) 
**Kilted** | [`kilted`](https://github.com/swri-robotics/gps_umd/tree/ros2-devel) | [![CI](https://github.com/swri-robotics/gps_umd/actions/workflows/main.yml/badge.svg?branch=ros2-devel)](https://github.com/swri-robotics/gps_umd/blob/ros2-devel/.github/workflows/main.yml?branch=ros2-devel) <br /> [![ROS2 Build Farm](http://build.ros2.org/buildStatus/icon?job=Kdev__gps_umd__ubuntu_noble_amd64)](https://build.ros2.org/job/Kdev__gps_umd__ubuntu_noble_amd64/) | [gps_msgs](https://index.ros.org/p/gps_msgs/#kilted) <br /> [gps_tools](https://index.ros.org/p/gps_tools/#kilted) <br /> [gpsd_client](https://index.ros.org/p/gpsd_client/#kilted) 
**Lyrical** | [`lyrical`](https://github.com/swri-robotics/gps_umd/tree/ros2-devel) | [![CI](https://github.com/swri-robotics/gps_umd/actions/workflows/main.yml/badge.svg?branch=ros2-devel)](https://github.com/swri-robotics/gps_umd/blob/ros2-devel/.github/workflows/main.yml?branch=ros2-devel) <br /> [![ROS2 Build Farm](http://build.ros2.org/buildStatus/icon?job=Ldev__gps_umd__ubuntu_resolute_amd64)](https://build.ros2.org/job/Ldev__gps_umd__ubuntu_resolute_amd64/) | [gps_msgs](https://index.ros.org/p/gps_msgs/#lyrical) <br /> [gps_tools](https://index.ros.org/p/gps_tools/#lyrical) <br /> [gpsd_client](https://index.ros.org/p/gpsd_client/#lyrical) 
**Rolling** | [`rolling`](https://github.com/swri-robotics/gps_umd/tree/ros2-devel) | [![CI](https://github.com/swri-robotics/gps_umd/actions/workflows/main.yml/badge.svg?branch=ros2-devel)](https://github.com/swri-robotics/gps_umd/blob/ros2-devel/.github/workflows/main.yml?branch=ros2-devel) <br /> [![ROS2 Build Farm](http://build.ros2.org/buildStatus/icon?job=Rdev__gps_umd__ubuntu_resolute_amd64)](https://build.ros2.org/job/Rdev__gps_umd__ubuntu_resolute_amd64/) | [gps_msgs](https://index.ros.org/p/gps_msgs/#rolling) <br /> [gps_tools](https://index.ros.org/p/gps_tools/#rolling) <br /> [gpsd_client](https://index.ros.org/p/gpsd_client/#rollingRevise build status for ROS2 distributions) 

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
