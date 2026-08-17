gps_umd 
=======

This package contains messages for representing data from GPS devices and algorithms for manipulating it.

This branch converts the messages and algorithms in this repository to support ROS 2 Dashing.

There have been a few architectural changes; if you were using these packages in ROS1, note that the `gps_common` package has been split into two packages: `gps_msgs` contains only message definitions, and `gps_tools` contains the nodes and scripts that were in `gps_common`.  In addition, all of the C++ nodes that were in this repository have been converted into Components.

Build Status
--------

Item | **Humble** | **Jazzy** | **Kilted** | **Lyrical** | **Rolling**
:--- | :---: | :---: | :---: | :---: | :---:
Build status | [![CI](https://github.com/swri-robotics/gps_umd/actions/workflows/humble.yml/badge.svg?branch=ros2-devel)](https://github.com/swri-robotics/gps_umd/blob/ros2-devel/.github/workflows/humble.yml) <br /> [![ROS2 Build Farm](https://build.ros2.org/buildStatus/icon?job=Hdev__gps_umd__ubuntu_jammy_amd64)](https://build.ros2.org/job/Hdev__gps_umd__ubuntu_jammy_amd64/) | [![CI](https://github.com/swri-robotics/gps_umd/actions/workflows/jazzy.yml/badge.svg?branch=ros2-devel)](https://github.com/swri-robotics/gps_umd/blob/ros2-devel/.github/workflows/jazzy.yml) <br /> [![ROS2 Build Farm](https://build.ros2.org/buildStatus/icon?job=Jdev__gps_umd__ubuntu_noble_amd64)](https://build.ros2.org/job/Jdev__gps_umd__ubuntu_noble_amd64/) | [![CI](https://github.com/swri-robotics/gps_umd/actions/workflows/kilted.yml/badge.svg?branch=ros2-devel)](https://github.com/swri-robotics/gps_umd/blob/ros2-devel/.github/workflows/kilted.yml) <br /> [![ROS2 Build Farm](https://build.ros2.org/buildStatus/icon?job=Kdev__gps_umd__ubuntu_noble_amd64)](https://build.ros2.org/job/Kdev__gps_umd__ubuntu_noble_amd64/) | [![CI](https://github.com/swri-robotics/gps_umd/actions/workflows/lyrical.yml/badge.svg?branch=ros2-devel)](https://github.com/swri-robotics/gps_umd/blob/ros2-devel/.github/workflows/lyrical.yml) <br /> [![ROS2 Build Farm](https://build.ros2.org/buildStatus/icon?job=Ldev__gps_umd__ubuntu_resolute_amd64)](https://build.ros2.org/job/Ldev__gps_umd__ubuntu_resolute_amd64/) | [![CI](https://github.com/swri-robotics/gps_umd/actions/workflows/rolling.yml/badge.svg?branch=ros2-devel)](https://github.com/swri-robotics/gps_umd/blob/ros2-devel/.github/workflows/rolling.yml) <br /> [![ROS2 Build Farm](https://build.ros2.org/buildStatus/icon?job=Rdev__gps_umd__ubuntu_resolute_amd64)](https://build.ros2.org/job/Rdev__gps_umd__ubuntu_resolute_amd64/)
`gps_msgs` | [Released](https://index.ros.org/p/gps_msgs/#humble) | [Released](https://index.ros.org/p/gps_msgs/#jazzy) | [Released](https://index.ros.org/p/gps_msgs/#kilted) | [Released](https://index.ros.org/p/gps_msgs/#lyrical) | [Released](https://index.ros.org/p/gps_msgs/#rolling)
`gps_tools` | [Released](https://index.ros.org/p/gps_tools/#humble) | [Released](https://index.ros.org/p/gps_tools/#jazzy) | [Released](https://index.ros.org/p/gps_tools/#kilted) | [Released](https://index.ros.org/p/gps_tools/#lyrical) | [Released](https://index.ros.org/p/gps_tools/#rolling)
`gpsd_client` | [Released](https://index.ros.org/p/gpsd_client/#humble) | [Released](https://index.ros.org/p/gpsd_client/#jazzy) | [Released](https://index.ros.org/p/gpsd_client/#kilted) | [Released](https://index.ros.org/p/gpsd_client/#lyrical) | [Released](https://index.ros.org/p/gpsd_client/#rolling)

gpsd API version coverage
-------------------------

[![gpsd generator](https://github.com/swri-robotics/gps_umd/actions/workflows/gpsd_generator.yml/badge.svg?branch=ros2-devel)](https://github.com/swri-robotics/gps_umd/actions/workflows/gpsd_generator.yml)

`gpsd_client` is built and tested against a real libgps for every gpsd C API
`(MAJOR, MINOR)` pair it supports. Each pair has its own workflow, so a failure
identifies the API version directly -- in the Actions list, in the badge below,
and in the notification -- rather than as one row of a matrix:

| API | gpsd revision | Message | Status |
|---|---|---|---|
| **9.0** | `3.20` | `GPSExtendedRaw9v0` | [![gpsd API 9.0](https://github.com/swri-robotics/gps_umd/actions/workflows/gpsd_api_9v0.yml/badge.svg?branch=ros2-devel)](https://github.com/swri-robotics/gps_umd/actions/workflows/gpsd_api_9v0.yml) |
| **9.1** | `e5279ef52` *(unreleased)* | `GPSExtendedRaw9v1` | [![gpsd API 9.1](https://github.com/swri-robotics/gps_umd/actions/workflows/gpsd_api_9v1.yml/badge.svg?branch=ros2-devel)](https://github.com/swri-robotics/gps_umd/actions/workflows/gpsd_api_9v1.yml) |
| **10.0** | `3.21` | `GPSExtendedRaw10v0` | [![gpsd API 10.0](https://github.com/swri-robotics/gps_umd/actions/workflows/gpsd_api_10v0.yml/badge.svg?branch=ros2-devel)](https://github.com/swri-robotics/gps_umd/actions/workflows/gpsd_api_10v0.yml) |
| **10.1** | `42f816d59` *(unreleased)* | `GPSExtendedRaw10v1` | [![gpsd API 10.1](https://github.com/swri-robotics/gps_umd/actions/workflows/gpsd_api_10v1.yml/badge.svg?branch=ros2-devel)](https://github.com/swri-robotics/gps_umd/actions/workflows/gpsd_api_10v1.yml) |
| **11.0** | `3.22` | `GPSExtendedRaw11v0` | [![gpsd API 11.0](https://github.com/swri-robotics/gps_umd/actions/workflows/gpsd_api_11v0.yml/badge.svg?branch=ros2-devel)](https://github.com/swri-robotics/gps_umd/actions/workflows/gpsd_api_11v0.yml) |
| **12.0** | `3.23.1` | `GPSExtendedRaw12v0` | [![gpsd API 12.0](https://github.com/swri-robotics/gps_umd/actions/workflows/gpsd_api_12v0.yml/badge.svg?branch=ros2-devel)](https://github.com/swri-robotics/gps_umd/actions/workflows/gpsd_api_12v0.yml) |
| **13.0** | `264e808c6` *(unreleased)* | `GPSExtendedRaw13v0` | [![gpsd API 13.0](https://github.com/swri-robotics/gps_umd/actions/workflows/gpsd_api_13v0.yml/badge.svg?branch=ros2-devel)](https://github.com/swri-robotics/gps_umd/actions/workflows/gpsd_api_13v0.yml) |
| **14.0** | `3.26.1` | `GPSExtendedRaw14v0` | [![gpsd API 14.0](https://github.com/swri-robotics/gps_umd/actions/workflows/gpsd_api_14v0.yml/badge.svg?branch=ros2-devel)](https://github.com/swri-robotics/gps_umd/actions/workflows/gpsd_api_14v0.yml) |
| **16.0** | `3.27.3` | `GPSExtendedRaw16v0` | [![gpsd API 16.0](https://github.com/swri-robotics/gps_umd/actions/workflows/gpsd_api_16v0.yml/badge.svg?branch=ros2-devel)](https://github.com/swri-robotics/gps_umd/actions/workflows/gpsd_api_16v0.yml) |
| **16.1** | `3.27.5` | `GPSExtendedRaw16v1` | [![gpsd API 16.1](https://github.com/swri-robotics/gps_umd/actions/workflows/gpsd_api_16v1.yml/badge.svg?branch=ros2-devel)](https://github.com/swri-robotics/gps_umd/actions/workflows/gpsd_api_16v1.yml) |

The three unreleased pairs shipped in no gpsd release and are pinned to the last
commit at which each was current; they are reachable only by building a gpsd git
checkout.

All ten call the same reusable workflow
(`.github/workflows/gpsd_api_shared.yml`), and the per-version files are
generated by `tools/generate_raw_msgs.py` from the same manifest that produces
the messages -- so adding an API version creates its workflow too, and the
`gpsd generator` check above fails if the tree is out of date.

Because each builds gpsd from source, they run on pushes to `ros2-devel`, on
pull requests that touch the packages or the generator, and on manual dispatch.
On failure the build and test logs upload as a `logs-api-<version>` artifact.

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
`publish_gpsd_raw` | bool | `false` | Also publish a near-verbatim mirror of gpsd's `gps_data_t` on `gpsd_raw`, in a message named after the libgps API this package was built against (see below). Off by default: the message is much larger than `GPSFix`, and neither the publisher nor its parser is created unless this is set.

These are the node's built-in defaults, used when a parameter is not set.
They match the config file shipped in `gpsd_client/config/gpsd_client.yaml`,
which is what `gpsd_client-launch.py` loads, so launching from that file and
instantiating the component directly behave the same.

Raw gpsd messages
-----------------

With `publish_gpsd_raw` set, `gpsd_client` also publishes everything gpsd
reports, as close to verbatim as a ROS message can be, on `gpsd_raw`.

The message type is named after the gpsd C API version the package was compiled
against: `gps_extended_msgs/GPSExtendedRaw<MAJOR>v<MINOR>`, from `GPSD_API_MAJOR_VERSION` and
`GPSD_API_MINOR_VERSION` in `gps.h`. Building against gpsd 3.27.5 gives
`GPSExtendedRaw16v1`; against gpsd 3.20, `GPSExtendedRaw9v0`. The node logs which one it
selected at startup. All of the message types are always built, so `gps_extended_msgs`
has no dependency on gpsd; only `gpsd_client` cares which libgps is present.

These live in their own package rather than in `gps_msgs`. There are roughly
900 of them and they take minutes to build, whereas `gps_msgs` is the small,
long-released package carrying `GPSFix` and `GPSStatus`; keeping them apart
leaves that build unchanged for everyone who does not want the raw data.

The messages and their parsers are generated from gpsd's own `gps.h` by
`tools/generate_raw_msgs.py`; see `docs/gpsd-raw-messages-plan.md` for the
design and the reasoning behind the version handling.

Points worth knowing before subscribing:

* **Fields move between versions.** The fix status lives in `status` on API 9
  and in `fix.status` from API 10 on, because that is where gpsd moved it. The
  raw message mirrors its own version rather than normalising, which is the
  point of having one type per version.
* **`NaN` means "unknown", not zero.** gpsd uses `NaN` as its no-value sentinel
  throughout, and it is preserved rather than replaced with `0.0`.
* **`set` is the report mask, copied undecoded.** Test it with the `SET_*`
  constants on the message, e.g. `msg.set & GPSExtendedRaw16v1::SET_LATLON`. The
  constants are gpsd's own bit values with the name reversed (`LATLON_SET`
  becomes `SET_LATLON`) because `gps.h` defines the gpsd spellings as
  preprocessor macros. Bit positions have never been renumbered, so mask tests
  are portable across versions even though the message types are not.
* **AIS is not carried.** `struct ais_t` is out of scope. `SET_AIS` is still
  defined and `set` still carries the bit, so `msg.set & SET_AIS` tells you
  gpsd reported AIS data this message does not include.
* **`skyview` is trimmed to `satellites_visible`.** gpsd's array is a fixed
  140 or 184 entries depending on version; only the valid prefix is published.

### An empty `skyview` alongside a non-zero `satellites_used`

This combination looks like a bug in this package and is not, so it is worth
recognising before you go looking for one.

Some receivers emit dilution-of-precision updates without a satellite list. gpsd
forwards those as a `SKY` report containing only `hdop`/`uSat` and friends — no
`satellites` array, and no `nSat` field. On receiving one, libgps clears its
skyview and drops `SATELLITE_SET` from the report mask. The result, faithfully
mirrored into the message:

```
satellites_visible : 0
satellites_used    : 12
skyview            : []
set & SET_SATELLITE: false
```

The `satellites_used` count is real, not left over: gpsd parses the report's
`uSat` field straight into it (gpsd 3.26.1 and newer). The receiver is saying it
used 12 satellites for the solution without listing which ones. On older gpsd,
which has no `uSat`, the same report leaves `satellites_used` at whatever the
last full `SKY` set, because libgps returns before resetting it — so there the
value genuinely can be stale.

Either way, `satellites_visible` and `skyview` agree with each other and are
simply empty. The reliable test for "does this message actually carry a
skyview" is the mask:

```cpp
if (msg.set & gps_extended_msgs::msg::GPSExtendedRaw16v1::SET_SATELLITE) {
  // skyview and satellites_visible are meaningful for this report
}
```

This is gpsd's behaviour rather than a translation artifact — `gpsd_client` does
not second-guess it, because a topic called "raw" that quietly repaired its
input would be worse. It also is not specific to a gpsd version; it depends on
what the receiver sends.

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
