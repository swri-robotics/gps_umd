from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus
from gps_msgs.msg import GPSFix
from gps_msgs.msg import GPSStatus


def navsatfix_to_gpsfix(navsat_msg):
    # Convert sensor_msgs/NavSatFix messages to gps_msgs/GPSFix messages
    gpsfix_msg = GPSFix()
    gpsfix_msg.header = navsat_msg.header
    gpsfix_msg.status.status = navsat_msg.status.status

    gpsfix_msg.status.motion_source = GPSStatus.SOURCE_NONE
    gpsfix_msg.status.orientation_source = GPSStatus.SOURCE_NONE
    gpsfix_msg.status.position_source = GPSStatus.SOURCE_NONE
    if ((navsat_msg.status.service & NavSatStatus.SERVICE_GPS) or
            (navsat_msg.status.service & NavSatStatus.SERVICE_GLONASS) or
            (navsat_msg.status.service & NavSatStatus.SERVICE_GALILEO)):
        gpsfix_msg.status.motion_source = \
            gpsfix_msg.status.motion_source | GPSStatus.SOURCE_GPS
        gpsfix_msg.status.orientation_source = \
            gpsfix_msg.status.orientation_source | GPSStatus.SOURCE_GPS
        gpsfix_msg.status.position_source = \
            gpsfix_msg.status.position_source | GPSStatus.SOURCE_GPS

    if navsat_msg.status.service & NavSatStatus.SERVICE_COMPASS:
        gpsfix_msg.status.orientation_source = \
            gpsfix_msg.status.orientation_source | GPSStatus.SOURCE_MAGNETIC

    gpsfix_msg.latitude = navsat_msg.latitude
    gpsfix_msg.longitude = navsat_msg.longitude
    gpsfix_msg.altitude = navsat_msg.altitude
    gpsfix_msg.position_covariance = navsat_msg.position_covariance
    gpsfix_msg.position_covariance_type = navsat_msg.position_covariance_type

    return gpsfix_msg


def gpsfix_to_navsatfix(gpsfix_msg):
    # Convert gps_msgs/GPSFix messages to sensor_msgs/NavSatFix messages
    navsat_msg = NavSatFix()
    navsat_msg.header = gpsfix_msg.header

    # Caution: GPSFix has defined some additional status constants, which are
    # not defined in NavSatFix.
    navsat_msg.status.status = gpsfix_msg.status.status

    navsat_msg.status.service = 0
    if gpsfix_msg.status.position_source & GPSStatus.SOURCE_GPS:
        navsat_msg.status.service = \
            navsat_msg.status.service | NavSatStatus.SERVICE_GPS
    if gpsfix_msg.status.orientation_source & GPSStatus.SOURCE_MAGNETIC:
        navsat_msg.status.service = \
            navsat_msg.status.service | NavSatStatus.SERVICE_COMPASS

    navsat_msg.latitude = gpsfix_msg.latitude
    navsat_msg.longitude = gpsfix_msg.longitude
    navsat_msg.altitude = gpsfix_msg.altitude
    navsat_msg.position_covariance = gpsfix_msg.position_covariance
    navsat_msg.position_covariance_type = gpsfix_msg.position_covariance_type

    return navsat_msg
