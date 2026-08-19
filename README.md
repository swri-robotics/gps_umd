gps_umd 
=======

This package contains messages for representing data from GPS devices and algorithms for manipulating it. It uses GPSd as the underlying method of reading data from GPS receivers.

Build Status
--------

Item | **Humble** | **Jazzy** | **Kilted** | **Lyrical** | **Rolling**
:--- | :---: | :---: | :---: | :---: | :---:
Build status | [![CI](https://github.com/swri-robotics/gps_umd/actions/workflows/humble.yml/badge.svg?branch=ros2-devel)](https://github.com/swri-robotics/gps_umd/blob/ros2-devel/.github/workflows/humble.yml) <br /> [![ROS2 Build Farm](https://build.ros2.org/buildStatus/icon?job=Hdev__gps_umd__ubuntu_jammy_amd64)](https://build.ros2.org/job/Hdev__gps_umd__ubuntu_jammy_amd64/) | [![CI](https://github.com/swri-robotics/gps_umd/actions/workflows/jazzy.yml/badge.svg?branch=ros2-devel)](https://github.com/swri-robotics/gps_umd/blob/ros2-devel/.github/workflows/jazzy.yml) <br /> [![ROS2 Build Farm](https://build.ros2.org/buildStatus/icon?job=Jdev__gps_umd__ubuntu_noble_amd64)](https://build.ros2.org/job/Jdev__gps_umd__ubuntu_noble_amd64/) | [![CI](https://github.com/swri-robotics/gps_umd/actions/workflows/kilted.yml/badge.svg?branch=ros2-devel)](https://github.com/swri-robotics/gps_umd/blob/ros2-devel/.github/workflows/kilted.yml) <br /> [![ROS2 Build Farm](https://build.ros2.org/buildStatus/icon?job=Kdev__gps_umd__ubuntu_noble_amd64)](https://build.ros2.org/job/Kdev__gps_umd__ubuntu_noble_amd64/) | [![CI](https://github.com/swri-robotics/gps_umd/actions/workflows/lyrical.yml/badge.svg?branch=ros2-devel)](https://github.com/swri-robotics/gps_umd/blob/ros2-devel/.github/workflows/lyrical.yml) <br /> [![ROS2 Build Farm](https://build.ros2.org/buildStatus/icon?job=Ldev__gps_umd__ubuntu_resolute_amd64)](https://build.ros2.org/job/Ldev__gps_umd__ubuntu_resolute_amd64/) | [![CI](https://github.com/swri-robotics/gps_umd/actions/workflows/rolling.yml/badge.svg?branch=ros2-devel)](https://github.com/swri-robotics/gps_umd/blob/ros2-devel/.github/workflows/rolling.yml) <br /> [![ROS2 Build Farm](https://build.ros2.org/buildStatus/icon?job=Rdev__gps_umd__ubuntu_resolute_amd64)](https://build.ros2.org/job/Rdev__gps_umd__ubuntu_resolute_amd64/)
`gps_msgs` | [Released](https://index.ros.org/p/gps_msgs/#humble) | [Released](https://index.ros.org/p/gps_msgs/#jazzy) | [Released](https://index.ros.org/p/gps_msgs/#kilted) | [Released](https://index.ros.org/p/gps_msgs/#lyrical) | [Released](https://index.ros.org/p/gps_msgs/#rolling)
`gps_tools` | [Released](https://index.ros.org/p/gps_tools/#humble) | [Released](https://index.ros.org/p/gps_tools/#jazzy) | [Released](https://index.ros.org/p/gps_tools/#kilted) | [Released](https://index.ros.org/p/gps_tools/#lyrical) | [Released](https://index.ros.org/p/gps_tools/#rolling)
`gpsd_client` | [Released](https://index.ros.org/p/gpsd_client/#humble) | [Released](https://index.ros.org/p/gpsd_client/#jazzy) | [Released](https://index.ros.org/p/gpsd_client/#kilted) | [Released](https://index.ros.org/p/gpsd_client/#lyrical) | [Released](https://index.ros.org/p/gpsd_client/#rolling)

GPSd Integration and Usage
-------------------------

By default, this package is built against the version of GPSd shipped with the Linux distribution the underlying ROS 2 software is built against. However, the `gps_umd` package supports building against arbitrary versions of GPSd and matching its behavior to the GPSd API version supplied by the library.

Depending on the GPS receiver, GPSd is capable of providing a great deal of information. By default, the package publishes a condensed version of this data as a `gps_msgs/GPSFix` message. Alternatively, the information may be published as a standard ROS 2 `sensor_msgs/NavSatFix` message. These messages provide the localization and error estimates that most robots need to operate in most conditions.

`gps_umd` can also publish the entire set of information provided by GPSd. These messages are in the `gps_extended_msgs` package and published as `GPSDRaw<MAJOR>v<MINOR>` types, where `<MAJOR>` and `<MINOR>` correspond to the GPSd API version that provides the information. The table below lists the currently support set of GPSd and API versions the system is built and tested against.

[![GPSd generator](https://github.com/swri-robotics/gps_umd/actions/workflows/gpsd_generator.yml/badge.svg?branch=ros2-devel)](https://github.com/swri-robotics/gps_umd/actions/workflows/gpsd_generator.yml)

| API | GPSd revision | Message | Status |
|---|---|---|---|
| **9.0** | `3.20` | `GPSDRaw9v0` | [![GPSd API 9.0](https://github.com/swri-robotics/gps_umd/actions/workflows/gpsd_api_9v0.yml/badge.svg?branch=ros2-devel)](https://github.com/swri-robotics/gps_umd/actions/workflows/gpsd_api_9v0.yml) |
| **9.1** | `e5279ef52` *(unreleased)* | `GPSDRaw9v1` | [![GPSd API 9.1](https://github.com/swri-robotics/gps_umd/actions/workflows/gpsd_api_9v1.yml/badge.svg?branch=ros2-devel)](https://github.com/swri-robotics/gps_umd/actions/workflows/gpsd_api_9v1.yml) |
| **10.0** | `3.21` | `GPSDRaw10v0` | [![GPSd API 10.0](https://github.com/swri-robotics/gps_umd/actions/workflows/gpsd_api_10v0.yml/badge.svg?branch=ros2-devel)](https://github.com/swri-robotics/gps_umd/actions/workflows/gpsd_api_10v0.yml) |
| **10.1** | `42f816d59` *(unreleased)* | `GPSDRaw10v1` | [![GPSd API 10.1](https://github.com/swri-robotics/gps_umd/actions/workflows/gpsd_api_10v1.yml/badge.svg?branch=ros2-devel)](https://github.com/swri-robotics/gps_umd/actions/workflows/gpsd_api_10v1.yml) |
| **11.0** | `3.22` | `GPSDRaw11v0` | [![GPSd API 11.0](https://github.com/swri-robotics/gps_umd/actions/workflows/gpsd_api_11v0.yml/badge.svg?branch=ros2-devel)](https://github.com/swri-robotics/gps_umd/actions/workflows/gpsd_api_11v0.yml) |
| **12.0** | `3.23.1` | `GPSDRaw12v0` | [![GPSd API 12.0](https://github.com/swri-robotics/gps_umd/actions/workflows/gpsd_api_12v0.yml/badge.svg?branch=ros2-devel)](https://github.com/swri-robotics/gps_umd/actions/workflows/gpsd_api_12v0.yml) |
| **13.0** | `264e808c6` *(unreleased)* | `GPSDRaw13v0` | [![GPSd API 13.0](https://github.com/swri-robotics/gps_umd/actions/workflows/gpsd_api_13v0.yml/badge.svg?branch=ros2-devel)](https://github.com/swri-robotics/gps_umd/actions/workflows/gpsd_api_13v0.yml) |
| **14.0** | `3.26.1` | `GPSDRaw14v0` | [![GPSd API 14.0](https://github.com/swri-robotics/gps_umd/actions/workflows/gpsd_api_14v0.yml/badge.svg?branch=ros2-devel)](https://github.com/swri-robotics/gps_umd/actions/workflows/gpsd_api_14v0.yml) |
| **16.0** | `3.27.3` | `GPSDRaw16v0` | [![GPSd API 16.0](https://github.com/swri-robotics/gps_umd/actions/workflows/gpsd_api_16v0.yml/badge.svg?branch=ros2-devel)](https://github.com/swri-robotics/gps_umd/actions/workflows/gpsd_api_16v0.yml) |
| **16.1** | `3.27.5` | `GPSDRaw16v1` | [![GPSd API 16.1](https://github.com/swri-robotics/gps_umd/actions/workflows/gpsd_api_16v1.yml/badge.svg?branch=ros2-devel)](https://github.com/swri-robotics/gps_umd/actions/workflows/gpsd_api_16v1.yml) |

Three GPSd API versions are not matched to a GPSd version. These GPSd versions are given as their commit hash instead. To add more version of the GPSd API, see: [docs/adding-a-gpsd-api-version.md](docs/adding-a-gpsd-api-version.md).

gpsd_client Parameters
----------------------

The `gpsd_client::GPSDClientComponent` node accepts the following parameters. Some are used to configure GPSd, and others are used to work around quirks of GPSd with certain hardware. In general, `gps_umd` attempts to retain the behavior of GPSd as much as possible, and deviate only to match ROS 2 conventions and robot needs.

Parameter | Type | Default | Description
:-------- | :--- | :------ | :----------
`host` | string | `localhost` | Hostname or address of the GPSd server to connect to.
`port` | int | `2947` | TCP port of the GPSd server.
`frame_id` | string | `gps` | `frame_id` set on the header of published `GPSFix` and `NavSatFix` messages.
`publish_rate` | int | `10` | How often, in Hz, to poll GPSd and publish. Values `<= 0` are rejected with a warning and fall back to 1 Hz.
`use_gps_time` | bool | `true` | Stamp `NavSatFix` messages with the time reported by the GPS receiver instead of the current ROS time.
`check_fix_by_variance` | bool | `false` | Discard fixes whose reported variances (`epx`/`epy`/`epv`) are not finite. GPSd reports a status of OK even when there is no current fix, as long as there was one previously; this rejects those stale results.
`override_augmentation_source` | bool | `false` | When GPSd reports a DGPS fix, always report it as an SBAS fix, whether or not a satellite with an SBAS ID was used in the solution. Useful for receivers that apply SBAS corrections without listing the SBAS satellite in their skyview. Affects both `NavSatFix` and `GPSFix` status.
`publish_gpsd_raw` | bool | `false` | Also publish a near-verbatim mirror of GPSd's `gps_data_t` on `gpsd_raw`, in a message named after the libgps API this package was built against (see below). Off by default: the message is much larger than `GPSFix`, and neither the publisher nor its parser is created unless this is set.
`publish_gpsd_json` | bool | `false` | Publish every report GPSd sends on `gpsd_json`, as the raw JSON line. Carries more than `gpsd_raw` can: libgps decodes 17 report classes and silently drops the rest, so RTCM and SUBFRAME reach subscribers only here.

These node defaults can be overriden by setting a parameter. The file `gpsd_client/config/gpsd_client.yaml` contains these parameters as well. The launch file `gpsd_client-launch.py` can load these YAML files for convenience.

Raw GPSd Details
-----------------

With `publish_gpsd_raw` set, `gpsd_client` publishes everything GPSd reports on
`gpsd_raw`, as close to verbatim as a ROS message allows. The type is named
after the GPSd C API the package compiled against —
`gps_extended_msgs/GPSDRaw<MAJOR>v<MINOR>` — and the node logs which one it
selected at startup. `publish_gpsd_json` adds `gpsd_json`, carrying every
report as the raw JSON line — including RTCM and SUBFRAME, which reach a client
no other way.

`tools/generate_raw_msgs.py` automatically converts the data structures in GPSd to these messages and their parsers.

These messages are complex due to the volume of information contained within them and how C structs and unions are mapped to ROS 2 message datatypes. There are also bugs and quirks in different GPSd versions that further complicate this translation. For more information, see the documentation in these files.

* [docs/gpsd-raw-message-structure.md](docs/gpsd-raw-message-structure.md) message layout and what changes between API versions
* [docs/gpsd-quirks.md](docs/gpsd-quirks.md) — GPSd behaviours worth knowing before subscribing


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

### Disclaimer
This project is not affiliated with the GPSd project. `gps_umd` utilizes the GPSd library as an interface to GPS receivers but is not a part of the GPSd project itself.
