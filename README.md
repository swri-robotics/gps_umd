gps_umd 
=======

This package contains messages for representing data from GPS devices and algorithms for manipulating it.

This branch converts the messages and algorithms in this repository to support ROS 2 Dashing.

There have been a few architectural changes; if you were using these packages in ROS1, note that the `gps_common` package has been split into two packages: `gps_msgs` contains only message definitions, and `gps_tools` contains the nodes and scripts that were in `gps_common`.  In addition, all of the C++ nodes that were in this repository have been converted into Components.

Build Status
--------
Continuous integration status

[![CI](https://github.com/swri-robotics/gps_umd/actions/workflows/main.yml/badge.svg?branch=ros2-devel)](https://github.com/swri-robotics/gps_umd/actions/workflows/main.yml?query=branch%3Aros2-devel)

Build farm status and released packages

ROS2 Distro | ROS2 Build Farm | Released packages
:---------: | :-------------: | :---------------:
**Humble** | [![ROS2 Build Farm](https://build.ros2.org/buildStatus/icon?job=Hdev__gps_umd__ubuntu_jammy_amd64)](https://build.ros2.org/job/Hdev__gps_umd__ubuntu_jammy_amd64/) | [gps_msgs](https://index.ros.org/p/gps_msgs/#humble) <br /> [gps_tools](https://index.ros.org/p/gps_tools/#humble) <br /> [gpsd_client](https://index.ros.org/p/gpsd_client/#humble)
**Jazzy** | [![ROS2 Build Farm](https://build.ros2.org/buildStatus/icon?job=Jdev__gps_umd__ubuntu_noble_amd64)](https://build.ros2.org/job/Jdev__gps_umd__ubuntu_noble_amd64/) | [gps_msgs](https://index.ros.org/p/gps_msgs/#jazzy) <br /> [gps_tools](https://index.ros.org/p/gps_tools/#jazzy) <br /> [gpsd_client](https://index.ros.org/p/gpsd_client/#jazzy)
**Kilted** | [![ROS2 Build Farm](https://build.ros2.org/buildStatus/icon?job=Kdev__gps_umd__ubuntu_noble_amd64)](https://build.ros2.org/job/Kdev__gps_umd__ubuntu_noble_amd64/) | [gps_msgs](https://index.ros.org/p/gps_msgs/#kilted) <br /> [gps_tools](https://index.ros.org/p/gps_tools/#kilted) <br /> [gpsd_client](https://index.ros.org/p/gpsd_client/#kilted)
**Lyrical** | [![ROS2 Build Farm](https://build.ros2.org/buildStatus/icon?job=Ldev__gps_umd__ubuntu_resolute_amd64)](https://build.ros2.org/job/Ldev__gps_umd__ubuntu_resolute_amd64/) | [gps_msgs](https://index.ros.org/p/gps_msgs/#lyrical) <br /> [gps_tools](https://index.ros.org/p/gps_tools/#lyrical) <br /> [gpsd_client](https://index.ros.org/p/gpsd_client/#lyrical)
**Rolling** | [![ROS2 Build Farm](https://build.ros2.org/buildStatus/icon?job=Rdev__gps_umd__ubuntu_resolute_amd64)](https://build.ros2.org/job/Rdev__gps_umd__ubuntu_resolute_amd64/) | [gps_msgs](https://index.ros.org/p/gps_msgs/#rolling) <br /> [gps_tools](https://index.ros.org/p/gps_tools/#rolling) <br /> [gpsd_client](https://index.ros.org/p/gpsd_client/#rolling)

gpsd_client Parameters
----------------------

The `gpsd_client::GPSDClientComponent` node accepts the following parameters:

Parameter | Type | Default | Description
:-------- | :--- | :------ | :----------
`host` | string | `localhost` | Hostname or address of the gpsd server to connect to.
`port` | int | `2947` | TCP port of the gpsd server.
`frame_id` | string | `gps` | `frame_id` set on the header of published `GPSFix` and `NavSatFix` messages.
`publish_rate` | int | `10` | How often, in Hz, to poll gpsd and publish. Values `<= 0` are rejected with a warning and fall back to 1 Hz.
`use_gps_time` | bool | `true` | Stamp `NavSatFix` messages with the time reported by the GPS receiver instead of the current ROS time.
`check_fix_by_variance` | bool | `false` | Discard fixes whose reported variances (`epx`/`epy`/`epv`) are not finite. gpsd reports a status of OK even when there is no current fix, as long as there was one previously; this rejects those stale results.
`override_augmentation_source` | bool | `false` | When gpsd reports a DGPS fix, always report it as an SBAS fix, whether or not a satellite with an SBAS ID was used in the solution. Useful for receivers that apply SBAS corrections without listing the SBAS satellite in their skyview. Affects both `NavSatFix` and `GPSFix` status.

These are the node's built-in defaults, used when a parameter is not set.
They match the config file shipped in `gpsd_client/config/gpsd_client.yaml`,
which is what `gpsd_client-launch.py` loads, so launching from that file and
instantiating the component directly behave the same.

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
