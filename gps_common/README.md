## Experimental Package

This package is a space to stage messages and common GPS-processing
routines that are undergoing a standardization process. Its contents
will probably be moved into [ros-pkg](ros-pkg) once they've matured.

## Messages

gps_common defines two common messages for GPS drivers to output:
[gps_common/msg/GPSFix](https://docs.ros.org/en/rolling/p/gps_common/msg/GPSFix.html)
and
[gps_common/msg/GPSStatus](https://docs.ros.org/en/rolling/p/gps_common/msg/GPSStatus.html).

In most cases, these messages should be published simultaneously, with
identical timestamps.

## Nodes

### utm_odometry_node

`utm_odometry_node` converts latitude-longitude readings into UTM
odometry

#### Subscribed Topics

- `fix`
  ([sensor_msgs/msg/NavSatFix](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/NavSatFix.html)):
  GPS measurement and status

#### Published Topics

- `odom`
  ([nav_msgs/msg/Odometry](https://docs.ros.org/en/rolling/p/nav_msgs/msg/Odometry.html)):
  UTM-encoded position

#### Parameters

- `~rot_covariance` (`double`, default: `99999`): Variance (in meters)
  to specify for rotational measurements
- `~frame_id` (`string`, default: Copy `frame_id` from `fix`
  message): Frame to specify in header of outgoing Odometry message
- `~child_frame_id` (`string`): Child frame to specify in header of
  outgoing Odometry message

## Tutorial

See [gpsd_client](https://index.ros.org/p/gpsd_client) for an example of
a sender node that uses this package's messages.
