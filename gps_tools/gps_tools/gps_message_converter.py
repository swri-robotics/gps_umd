# Copyright (c) 2019, Southwest Research Institute® (SwRI®)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Southwest Research Institute® (SwRI®) nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from gps_msgs.msg import GPSFix
from gps_msgs.msg import GPSStatus
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus

# GPSStatus values that NavSatStatus does not define, mapped the way
# gpsd_client maps GPSd's statuses: differential and RTK fixes are ground-based
# augmentation, and WAAS is satellite-based.
_NAVSAT_STATUS = {
    GPSStatus.STATUS_DGPS_FIX: NavSatStatus.STATUS_GBAS_FIX,
    GPSStatus.STATUS_RTK_FIX: NavSatStatus.STATUS_GBAS_FIX,
    GPSStatus.STATUS_RTK_FLOAT: NavSatStatus.STATUS_GBAS_FIX,
    GPSStatus.STATUS_WAAS_FIX: NavSatStatus.STATUS_SBAS_FIX,
}

# Every NavSatStatus service is a satellite constellation. SERVICE_COMPASS is
# BeiDou, once called Compass, not a magnetic compass, so it is a GNSS source
# like the others and says nothing about orientation from magnetic sensors.
_GNSS_SERVICES = (NavSatStatus.SERVICE_GPS | NavSatStatus.SERVICE_GLONASS |
                  NavSatStatus.SERVICE_COMPASS | NavSatStatus.SERVICE_GALILEO)


def navsatfix_to_gpsfix(navsat_msg):
    # Convert sensor_msgs/NavSatFix messages to gps_msgs/GPSFix messages
    gpsfix_msg = GPSFix()
    gpsfix_msg.header = navsat_msg.header
    gpsfix_msg.status.status = navsat_msg.status.status

    gpsfix_msg.status.motion_source = GPSStatus.SOURCE_NONE
    gpsfix_msg.status.orientation_source = GPSStatus.SOURCE_NONE
    gpsfix_msg.status.position_source = GPSStatus.SOURCE_NONE
    if navsat_msg.status.service & _GNSS_SERVICES:
        gpsfix_msg.status.motion_source = \
            gpsfix_msg.status.motion_source | GPSStatus.SOURCE_GPS
        gpsfix_msg.status.orientation_source = \
            gpsfix_msg.status.orientation_source | GPSStatus.SOURCE_GPS
        gpsfix_msg.status.position_source = \
            gpsfix_msg.status.position_source | GPSStatus.SOURCE_GPS

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

    navsat_msg.status.status = _NAVSAT_STATUS.get(
        gpsfix_msg.status.status, gpsfix_msg.status.status)

    navsat_msg.status.service = 0
    if gpsfix_msg.status.position_source & GPSStatus.SOURCE_GPS:
        navsat_msg.status.service = \
            navsat_msg.status.service | NavSatStatus.SERVICE_GPS

    navsat_msg.latitude = gpsfix_msg.latitude
    navsat_msg.longitude = gpsfix_msg.longitude
    navsat_msg.altitude = gpsfix_msg.altitude
    navsat_msg.position_covariance = gpsfix_msg.position_covariance
    navsat_msg.position_covariance_type = gpsfix_msg.position_covariance_type

    return navsat_msg
