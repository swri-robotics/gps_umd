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

"""Tests for bag_converter, against bags written with rosbag2_py."""

import os
import subprocess

from ament_index_python.packages import get_package_prefix
from gps_msgs.msg import GPSFix
from gps_msgs.msg import GPSStatus
from gps_tools.bag_converter import convert_bag
import pytest
from rclpy.serialization import deserialize_message
from rclpy.serialization import serialize_message
import rosbag2_py
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus
from std_msgs.msg import String

BAG_CONVERTER = os.path.join(
    get_package_prefix('gps_tools'), 'lib', 'gps_tools', 'bag_converter')

NAVSATFIX = 'sensor_msgs/msg/NavSatFix'
GPSFIX = 'gps_msgs/msg/GPSFix'
STRING = 'std_msgs/msg/String'


def storage_ids():
    """sqlite3 everywhere, and mcap where its plugin is installed."""
    writers = rosbag2_py.get_registered_writers()
    return [storage for storage in ('sqlite3', 'mcap') if storage in writers]


def topic_metadata(topic_id, name, type_name):
    # The constructor gained a leading id after Humble.
    try:
        return rosbag2_py.TopicMetadata(topic_id, name, type_name, 'cdr')
    except TypeError:
        return rosbag2_py.TopicMetadata(name, type_name, 'cdr')


def navsatfix(i):
    msg = NavSatFix()
    msg.header.frame_id = 'gps'
    msg.status.status = NavSatStatus.STATUS_FIX
    msg.status.service = NavSatStatus.SERVICE_GPS
    msg.latitude = 29.45 + i
    msg.longitude = -98.61
    msg.altitude = 250.0
    return msg


def gpsfix(i):
    msg = GPSFix()
    msg.header.frame_id = 'gps'
    msg.status.status = GPSStatus.STATUS_DGPS_FIX
    msg.status.position_source = GPSStatus.SOURCE_GPS
    msg.latitude = 40.0 + i
    msg.longitude = -105.0
    return msg


def write_bag(uri, storage_id):
    """Write a bag with a NavSatFix, a GPSFix and a String topic."""
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=uri, storage_id=storage_id),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    writer.create_topic(topic_metadata(0, '/fix', NAVSATFIX))
    writer.create_topic(topic_metadata(1, '/extended_fix', GPSFIX))
    writer.create_topic(topic_metadata(2, '/chatter', STRING))
    for i in range(3):
        writer.write('/fix', serialize_message(navsatfix(i)), 1_000_000_000 + i)
    for i in range(2):
        writer.write('/extended_fix', serialize_message(gpsfix(i)), 2_000_000_000 + i)
    writer.write('/chatter', serialize_message(String(data='unchanged')), 3_000_000_000)
    del writer


def read_bag(uri):
    """Return {topic: type}, {topic: [(bytes, timestamp)]} and the storage id."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=uri, storage_id=''),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    messages = {}
    while reader.has_next():
        topic, data, timestamp = reader.read_next()[:3]
        messages.setdefault(topic, []).append((bytes(data), timestamp))
    return types, messages, reader.get_metadata().storage_identifier


@pytest.fixture(params=storage_ids())
def bag(request, tmp_path):
    uri = str(tmp_path / 'input')
    write_bag(uri, request.param)
    return uri, request.param


def test_converts_every_fix_topic_both_ways(bag, tmp_path):
    uri, storage_id = bag
    _, before, _ = read_bag(uri)
    output = str(tmp_path / 'output')
    converted = convert_bag(uri, output)
    assert converted == {'/fix': GPSFIX, '/extended_fix': NAVSATFIX}

    types, after, output_storage = read_bag(output)
    assert types == {'/fix': GPSFIX, '/extended_fix': NAVSATFIX, '/chatter': STRING}
    # The output keeps the input's storage format.
    assert output_storage == storage_id

    # Every message is still there, at the same time.
    for topic in before:
        assert [t for _, t in after[topic]] == [t for _, t in before[topic]]

    for i, (data, _) in enumerate(after['/fix']):
        msg = deserialize_message(data, GPSFix)
        assert msg.latitude == pytest.approx(29.45 + i)
        assert msg.status.position_source == GPSStatus.SOURCE_GPS
    for i, (data, _) in enumerate(after['/extended_fix']):
        msg = deserialize_message(data, NavSatFix)
        assert msg.latitude == pytest.approx(40.0 + i)
        assert msg.status.status == NavSatStatus.STATUS_GBAS_FIX  # from DGPS

    # Everything else is copied byte for byte.
    assert after['/chatter'] == before['/chatter']


def test_converts_only_the_named_topics(bag, tmp_path):
    uri, _ = bag
    output = str(tmp_path / 'output')
    # Names without a leading slash match, as on the command line.
    assert convert_bag(uri, output, topics=['fix']) == {'/fix': GPSFIX}
    types, after, _ = read_bag(output)
    assert types['/fix'] == GPSFIX
    assert types['/extended_fix'] == GPSFIX
    _, before, _ = read_bag(uri)
    assert after['/extended_fix'] == before['/extended_fix']


def test_refuses_an_existing_output(bag, tmp_path):
    uri, _ = bag
    output = tmp_path / 'output'
    output.mkdir()
    _, before, _ = read_bag(uri)
    with pytest.raises(FileExistsError):
        convert_bag(uri, str(output))
    # The input is untouched either way.
    assert read_bag(uri)[1] == before


@pytest.mark.parametrize('storage_id', storage_ids())
def test_writes_the_requested_storage(tmp_path, storage_id):
    uri = str(tmp_path / 'input')
    write_bag(uri, 'sqlite3')
    output = str(tmp_path / 'output')
    convert_bag(uri, output, storage_id=storage_id)
    assert read_bag(output)[2] == storage_id


def test_installed_script_converts_a_bag(tmp_path):
    uri = str(tmp_path / 'input')
    write_bag(uri, 'sqlite3')
    output = str(tmp_path / 'output')
    result = subprocess.run([BAG_CONVERTER, uri, output, '--topics', '/fix', '/nonexistent'],
                            capture_output=True, text=True, timeout=60)
    assert result.returncode == 0, result.stderr
    assert '/fix: converted to gps_msgs/msg/GPSFix' in result.stdout
    assert '/nonexistent is not a NavSatFix or GPSFix topic' in result.stderr
    assert read_bag(output)[0]['/fix'] == GPSFIX

    # Refusing an existing output is a usage error, not a traceback.
    again = subprocess.run([BAG_CONVERTER, uri, output], capture_output=True, text=True,
                           timeout=60)
    assert again.returncode == 2
    assert 'already exists' in again.stderr
    assert 'Traceback' not in again.stderr
