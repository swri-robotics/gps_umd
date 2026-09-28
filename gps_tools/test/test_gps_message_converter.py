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

"""Tests for gps_message_converter and the fix_translator node built on it."""

from importlib.machinery import SourceFileLoader
import os
import time
import unittest

from gps_msgs.msg import GPSFix
from gps_msgs.msg import GPSStatus
from gps_tools import gps_message_converter as converter
import rclpy
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus

FIX_TRANSLATOR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'nodes', 'fix_translator')


def make_navsatfix(status=NavSatStatus.STATUS_FIX, service=NavSatStatus.SERVICE_GPS):
    msg = NavSatFix()
    msg.header.stamp.sec = 1000
    msg.header.stamp.nanosec = 500
    msg.header.frame_id = 'gps'
    msg.status.status = status
    msg.status.service = service
    msg.latitude = 29.4241
    msg.longitude = -98.4936
    msg.altitude = 198.0
    msg.position_covariance = [float(i) for i in range(1, 10)]
    msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
    return msg


def make_gpsfix(status=GPSStatus.STATUS_FIX, position_source=GPSStatus.SOURCE_GPS,
                orientation_source=GPSStatus.SOURCE_NONE):
    msg = GPSFix()
    msg.header.stamp.sec = 1000
    msg.header.stamp.nanosec = 500
    msg.header.frame_id = 'gps'
    msg.status.status = status
    msg.status.position_source = position_source
    msg.status.orientation_source = orientation_source
    msg.latitude = 29.4241
    msg.longitude = -98.4936
    msg.altitude = 198.0
    msg.position_covariance = [float(i) for i in range(1, 10)]
    msg.position_covariance_type = GPSFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
    return msg


class NavSatFixToGpsFix(unittest.TestCase):

    def test_copies_the_header_position_and_covariance(self):
        gpsfix = converter.navsatfix_to_gpsfix(make_navsatfix())
        self.assertEqual(1000, gpsfix.header.stamp.sec)
        self.assertEqual(500, gpsfix.header.stamp.nanosec)
        self.assertEqual('gps', gpsfix.header.frame_id)
        self.assertEqual(29.4241, gpsfix.latitude)
        self.assertEqual(-98.4936, gpsfix.longitude)
        self.assertEqual(198.0, gpsfix.altitude)
        self.assertEqual([float(i) for i in range(1, 10)], list(gpsfix.position_covariance))
        self.assertEqual(GPSFix.COVARIANCE_TYPE_DIAGONAL_KNOWN, gpsfix.position_covariance_type)

    def test_keeps_the_status(self):
        # Every NavSatStatus value means the same thing in GPSStatus.
        for status in (NavSatStatus.STATUS_NO_FIX, NavSatStatus.STATUS_FIX,
                       NavSatStatus.STATUS_SBAS_FIX, NavSatStatus.STATUS_GBAS_FIX):
            with self.subTest(status=status):
                gpsfix = converter.navsatfix_to_gpsfix(make_navsatfix(status=status))
                self.assertEqual(status, gpsfix.status.status)

    def test_any_satellite_service_makes_gps_the_source(self):
        for service in (NavSatStatus.SERVICE_GPS, NavSatStatus.SERVICE_GLONASS,
                        NavSatStatus.SERVICE_GALILEO):
            with self.subTest(service=service):
                status = converter.navsatfix_to_gpsfix(make_navsatfix(service=service)).status
                self.assertEqual(GPSStatus.SOURCE_GPS, status.position_source)
                self.assertEqual(GPSStatus.SOURCE_GPS, status.motion_source)
                self.assertEqual(GPSStatus.SOURCE_GPS, status.orientation_source)

    def test_a_compass_adds_a_magnetic_orientation_source(self):
        service = NavSatStatus.SERVICE_GPS | NavSatStatus.SERVICE_COMPASS
        status = converter.navsatfix_to_gpsfix(make_navsatfix(service=service)).status
        self.assertEqual(GPSStatus.SOURCE_GPS | GPSStatus.SOURCE_MAGNETIC,
                         status.orientation_source)
        self.assertEqual(GPSStatus.SOURCE_GPS, status.position_source)

    def test_no_service_means_no_source(self):
        status = converter.navsatfix_to_gpsfix(make_navsatfix(service=0)).status
        self.assertEqual(GPSStatus.SOURCE_NONE, status.position_source)
        self.assertEqual(GPSStatus.SOURCE_NONE, status.motion_source)
        self.assertEqual(GPSStatus.SOURCE_NONE, status.orientation_source)


class GpsFixToNavSatFix(unittest.TestCase):

    def test_copies_the_header_position_and_covariance(self):
        navsat = converter.gpsfix_to_navsatfix(make_gpsfix())
        self.assertEqual(1000, navsat.header.stamp.sec)
        self.assertEqual(500, navsat.header.stamp.nanosec)
        self.assertEqual('gps', navsat.header.frame_id)
        self.assertEqual(29.4241, navsat.latitude)
        self.assertEqual(-98.4936, navsat.longitude)
        self.assertEqual(198.0, navsat.altitude)
        self.assertEqual([float(i) for i in range(1, 10)], list(navsat.position_covariance))
        self.assertEqual(NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN,
                         navsat.position_covariance_type)

    def test_maps_every_status_to_one_navsatstatus_defines(self):
        # The GPSStatus values past STATUS_GBAS_FIX have no NavSatStatus
        # counterpart, so they map the way gpsd_client maps GPSd's statuses:
        # differential and RTK fixes are ground-based augmentation, and WAAS
        # is satellite-based.
        expected = {
            GPSStatus.STATUS_NO_FIX: NavSatStatus.STATUS_NO_FIX,
            GPSStatus.STATUS_FIX: NavSatStatus.STATUS_FIX,
            GPSStatus.STATUS_SBAS_FIX: NavSatStatus.STATUS_SBAS_FIX,
            GPSStatus.STATUS_GBAS_FIX: NavSatStatus.STATUS_GBAS_FIX,
            GPSStatus.STATUS_DGPS_FIX: NavSatStatus.STATUS_GBAS_FIX,
            GPSStatus.STATUS_RTK_FIX: NavSatStatus.STATUS_GBAS_FIX,
            GPSStatus.STATUS_RTK_FLOAT: NavSatStatus.STATUS_GBAS_FIX,
            GPSStatus.STATUS_WAAS_FIX: NavSatStatus.STATUS_SBAS_FIX,
        }
        for status, navsat_status in expected.items():
            with self.subTest(status=status):
                navsat = converter.gpsfix_to_navsatfix(make_gpsfix(status=status))
                self.assertEqual(navsat_status, navsat.status.status)

    def test_maps_the_sources_to_services(self):
        navsat = converter.gpsfix_to_navsatfix(make_gpsfix(
            orientation_source=GPSStatus.SOURCE_MAGNETIC))
        self.assertEqual(NavSatStatus.SERVICE_GPS | NavSatStatus.SERVICE_COMPASS,
                         navsat.status.service)
        navsat = converter.gpsfix_to_navsatfix(make_gpsfix(
            position_source=GPSStatus.SOURCE_NONE))
        self.assertEqual(0, navsat.status.service)

    def test_round_trips_a_navsatfix(self):
        original = make_navsatfix(
            status=NavSatStatus.STATUS_SBAS_FIX,
            service=NavSatStatus.SERVICE_GPS | NavSatStatus.SERVICE_COMPASS)
        back = converter.gpsfix_to_navsatfix(converter.navsatfix_to_gpsfix(original))
        self.assertEqual(original.header, back.header)
        self.assertEqual(original.status, back.status)
        self.assertEqual(original.latitude, back.latitude)
        self.assertEqual(original.longitude, back.longitude)
        self.assertEqual(original.altitude, back.altitude)
        self.assertEqual(list(original.position_covariance), list(back.position_covariance))
        self.assertEqual(original.position_covariance_type, back.position_covariance_type)


class FixTranslatorNode(unittest.TestCase):
    """Runs fix_translator in this process and translates through its topics."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.module = SourceFileLoader('fix_translator', FIX_TRANSLATOR).load_module()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.translator = self.module.FixTranslator()
        self.node = rclpy.create_node('test_fix_translator')
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.translator)
        self.executor.add_node(self.node)

    def tearDown(self):
        self.executor.shutdown()
        self.node.destroy_node()
        self.translator.destroy_node()

    def translate(self, in_type, in_topic, msg, out_type, out_topic):
        # The translator's topics are transient local, so match them.
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        received = []
        self.node.create_subscription(out_type, out_topic, received.append, qos)
        pub = self.node.create_publisher(in_type, in_topic, qos)
        pub.publish(msg)
        deadline = time.monotonic() + 5.0
        while not received and time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.05)
        self.assertTrue(received, f'nothing arrived on {out_topic}')
        return received[0]

    def test_translates_a_navsatfix_to_a_gpsfix(self):
        gpsfix = self.translate(NavSatFix, 'navsat_fix_in', make_navsatfix(),
                                GPSFix, 'gps_fix_out')
        self.assertEqual(29.4241, gpsfix.latitude)
        self.assertEqual(GPSStatus.SOURCE_GPS, gpsfix.status.position_source)

    def test_translates_a_gpsfix_to_a_navsatfix(self):
        navsat = self.translate(GPSFix, 'gps_fix_in', make_gpsfix(),
                                NavSatFix, 'navsat_fix_out')
        self.assertEqual(-98.4936, navsat.longitude)
        self.assertEqual(NavSatStatus.SERVICE_GPS, navsat.status.service)
