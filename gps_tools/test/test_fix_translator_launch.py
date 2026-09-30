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

"""Launch fix_translator.launch.py and translate through it."""

# fix_translator is started the way `ros2 run` and `ros2 launch` find it, by
# package and executable name, so this also checks where it is installed.

import os
import time
import unittest

from gps_msgs.msg import GPSFix
from gps_msgs.msg import GPSStatus
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus

LAUNCH_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'launch', 'fix_translator.launch.py')


@pytest.mark.launch_test
def generate_test_description():
    translator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(LAUNCH_FILE),
        launch_arguments={
            'navsat_fix_in': 'test_navsat_in',
            'gps_fix_out': 'test_gps_out',
            'gps_fix_in': 'test_gps_in',
            'navsat_fix_out': 'test_navsat_out',
        }.items())
    return launch.LaunchDescription([
        translator,
        launch_testing.actions.ReadyToTest(),
    ])


class FixTranslatorLaunch(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_fix_translator_launch')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def translate(self, in_type, in_topic, msg, out_type, out_topic):
        # The translator's topics are transient local, so match them.
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        received = []
        sub = self.node.create_subscription(out_type, out_topic, received.append, qos)
        pub = self.node.create_publisher(in_type, in_topic, qos)
        try:
            # Publish until the translator, started in another process, has
            # discovered this node and answered.
            deadline = time.monotonic() + 20.0
            while not received and time.monotonic() < deadline:
                pub.publish(msg)
                rclpy.spin_once(self.node, timeout_sec=0.2)
        finally:
            self.node.destroy_publisher(pub)
            self.node.destroy_subscription(sub)
        self.assertTrue(received, f'nothing arrived on {out_topic}')
        return received[0]

    def test_translates_a_navsatfix_on_the_remapped_topics(self):
        msg = NavSatFix()
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude = 29.4241
        gpsfix = self.translate(NavSatFix, 'test_navsat_in', msg, GPSFix, 'test_gps_out')
        self.assertEqual(29.4241, gpsfix.latitude)
        self.assertEqual(GPSStatus.SOURCE_GPS, gpsfix.status.position_source)

    def test_translates_a_gpsfix_on_the_remapped_topics(self):
        msg = GPSFix()
        msg.status.position_source = GPSStatus.SOURCE_GPS
        msg.longitude = -98.4936
        navsat = self.translate(GPSFix, 'test_gps_in', msg, NavSatFix, 'test_navsat_out')
        self.assertEqual(-98.4936, navsat.longitude)
        self.assertEqual(NavSatStatus.SERVICE_GPS, navsat.status.service)


@launch_testing.post_shutdown_test()
class FixTranslatorShutdown(unittest.TestCase):

    def test_exits_cleanly(self, proc_info):
        # launch stops the node with SIGINT at the end of the test, which it
        # should handle as a requested shutdown and exit 0.
        launch_testing.asserts.assertExitCodes(proc_info)
