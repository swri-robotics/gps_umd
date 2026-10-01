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

"""Convert between sensor_msgs/NavSatFix and gps_msgs/GPSFix in a recorded bag."""

# Every NavSatFix topic becomes a GPSFix topic and every GPSFix topic a
# NavSatFix one, using the same conversions as fix_translator. Everything else
# is copied as it was recorded, byte for byte, so no message definitions are
# needed for it. Timestamps are kept, and the input bag is never modified.

import argparse
import os
import sys

from gps_msgs.msg import GPSFix
from gps_tools import gps_message_converter as converter
from rclpy.serialization import deserialize_message
from rclpy.serialization import serialize_message
import rosbag2_py
from sensor_msgs.msg import NavSatFix

NAVSATFIX = 'sensor_msgs/msg/NavSatFix'
GPSFIX = 'gps_msgs/msg/GPSFix'

# Input type: (input message class, conversion, output type).
CONVERSIONS = {
    NAVSATFIX: (NavSatFix, converter.navsatfix_to_gpsfix, GPSFIX),
    GPSFIX: (GPSFix, converter.gpsfix_to_navsatfix, NAVSATFIX),
}


def absolute_name(topic):
    """Return a topic name with its leading slash, as bags record them."""
    return topic if topic.startswith('/') else '/' + topic


def convert_bag(input_uri, output_uri, topics=None, storage_id=None):
    """
    Write a copy of a bag with its NavSatFix and GPSFix topics converted.

    :param input_uri: The bag to read.
    :param output_uri: Where to write the converted bag. Must not exist.
    :param topics: Only convert these topics; all others are copied. None
        converts every NavSatFix and GPSFix topic.
    :param storage_id: Storage plugin for the output, such as 'sqlite3' or
        'mcap'. None uses the input's.
    :return: A dict of converted topic names to their new types.
    """
    if os.path.exists(output_uri):
        raise FileExistsError(f'{output_uri} already exists')
    if topics is not None:
        topics = {absolute_name(name) for name in topics}

    converter_options = rosbag2_py.ConverterOptions('cdr', 'cdr')
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=input_uri, storage_id=''), converter_options)
    if storage_id is None:
        storage_id = reader.get_metadata().storage_identifier

    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=output_uri, storage_id=storage_id),
                converter_options)

    converted = {}
    for metadata in reader.get_all_topics_and_types():
        if metadata.type in CONVERSIONS and (topics is None or metadata.name in topics):
            converted[metadata.name] = CONVERSIONS[metadata.type]
            # The metadata is reused rather than rebuilt, because its
            # constructor differs between ROS distributions. A recorded type
            # hash describes the old type, so it is dropped.
            metadata.type = CONVERSIONS[metadata.type][2]
            if hasattr(metadata, 'type_description_hash'):
                metadata.type_description_hash = ''
        writer.create_topic(metadata)

    # read_next_ext() also returns the send timestamp, where this rosbag2 has
    # it; read_next() is all that older ones offer.
    read_next = getattr(reader, 'read_next_ext', reader.read_next)
    while reader.has_next():
        topic, data, *timestamps = read_next()
        if topic in converted:
            msg_type, convert, _ = converted[topic]
            data = serialize_message(convert(deserialize_message(data, msg_type)))
        writer.write(topic, data, *timestamps)

    del writer  # closes the output bag
    return {name: conversion[2] for name, conversion in converted.items()}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input_bag', help='the bag to read')
    parser.add_argument('output_bag', help='where to write the converted bag; must not exist')
    parser.add_argument('--topics', nargs='+', metavar='TOPIC',
                        help='only convert these topics (default: every NavSatFix and GPSFix '
                             'topic)')
    parser.add_argument('--storage-id', help="output storage plugin, such as 'sqlite3' or "
                                             "'mcap' (default: the input's)")
    args = parser.parse_args(argv)
    try:
        converted = convert_bag(args.input_bag, args.output_bag, args.topics, args.storage_id)
    except FileExistsError as error:
        parser.error(str(error))
    if args.topics:
        missing = {absolute_name(name) for name in args.topics} - set(converted)
        for name in sorted(missing):
            print(f'warning: {name} is not a NavSatFix or GPSFix topic in '
                  f'{args.input_bag}; copied unchanged', file=sys.stderr)
    for name, new_type in sorted(converted.items()):
        print(f'{name}: converted to {new_type}')
    return 0
