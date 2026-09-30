# Copyright (c) 2026, Southwest Research Institute® (SwRI®)
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

"""GPSFix and GPSStatus keep exactly their fields."""

# Recorded bags carry messages serialized with these fields, and a subscriber
# built from a message with a field added or removed never receives them: the
# type hash changes and playback fails silently. Constants and comments can
# change freely; they are not part of the serialized message or its hash.
# Anything that needs a new field wants a new message instead.

from gps_msgs.msg import GPSFix
from gps_msgs.msg import GPSStatus

GPS_STATUS_FIELDS = {
    'header': 'std_msgs/Header',
    'satellites_used': 'uint16',
    'satellite_used_prn': 'sequence<int32>',
    'satellites_visible': 'uint16',
    'satellite_visible_prn': 'sequence<int32>',
    'satellite_visible_z': 'sequence<int32>',
    'satellite_visible_azimuth': 'sequence<int32>',
    'satellite_visible_snr': 'sequence<int32>',
    'status': 'int16',
    'motion_source': 'uint16',
    'orientation_source': 'uint16',
    'position_source': 'uint16',
}

GPS_FIX_FIELDS = {
    'header': 'std_msgs/Header',
    'status': 'gps_msgs/GPSStatus',
    'latitude': 'double',
    'longitude': 'double',
    'altitude': 'double',
    'track': 'double',
    'speed': 'double',
    'climb': 'double',
    'pitch': 'double',
    'roll': 'double',
    'dip': 'double',
    'time': 'double',
    'gdop': 'double',
    'pdop': 'double',
    'hdop': 'double',
    'vdop': 'double',
    'tdop': 'double',
    'err': 'double',
    'err_horz': 'double',
    'err_vert': 'double',
    'err_track': 'double',
    'err_speed': 'double',
    'err_climb': 'double',
    'err_time': 'double',
    'err_pitch': 'double',
    'err_roll': 'double',
    'err_dip': 'double',
    'position_covariance': 'double[9]',
    'position_covariance_type': 'uint8',
}


def test_gps_status_keeps_its_fields():
    # Compared as lists, so reordering fields fails too: order is serialized.
    assert list(GPSStatus.get_fields_and_field_types().items()) == \
        list(GPS_STATUS_FIELDS.items())


def test_gps_fix_keeps_its_fields():
    assert list(GPSFix.get_fields_and_field_types().items()) == \
        list(GPS_FIX_FIELDS.items())
